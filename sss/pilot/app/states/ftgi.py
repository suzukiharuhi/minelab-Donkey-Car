"""FTG-i (Follow The Gap improved) による障害物回避状態.

depth image から 1D レンジプロファイルを生成し, 自由空間ギャップを列挙,
評価関数 U(θ) に基づいて進行方向を選択して ControlCommand を返す.

インタフェース:
	- BaseState を継承
	- enter()   : 状態開始時に呼ばれる
	- execute() : 毎フレーム, depth image から制御コマンドを生成
	- check_transition(): 状態遷移判定（本状態では常に None を返す）
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import os
from datetime import datetime

import cv2
import numpy as np
import rospy

from pilot.app.states.base_state import BaseState
from pilot.app.states.state_type import StateType
from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData
from pilot.app.core.commands import ControlCommand, CommandType
# from pilot.app.logging import SessionLogger


@dataclass
class FTGIConfig:
	"""FTG-i の各種パラメータ.

	数値はデフォルト値で, 後から rosparam 等で置き換えることを想定.
	"""

	# depth image から使用する領域（画像サイズに対する割合）
	v_min_frac: float = 0.30  # 高さ方向: 上端からの割合（0〜1）
	v_max_frac: float = 0.60
	u_min_frac: float = 0.20  # 幅方向: 左端からの割合（0〜1）
	u_max_frac: float = 0.80

	# 1D レンジプロファイルのサンプル数 K
	num_samples: int = 300  # 120〜240 程度を想定

	# レンジ集約
	use_trimmed_mean: bool = True  # True: 20% トリム平均, False: メディアン
	trim_ratio: float = 0.2

	# 1D 配列に対するメディアンフィルタ
	median_filter_ksize: int = 3

	# 自由空間判定しきい値 [mm]
	d_free: float = 100#2500.0#4000.0

	# ギャップ(自由空間区間)の最小幅
	min_gap_width_pixels: int = 10
	min_gap_width_ratio: float = 0.05  # K に対する割合

	# U(θ) の重み
	w_clearance: float = 0.8
	w_centering: float = 0.9
	w_goal: float = 0.0  # いまはゴール無し → 0
	w_steer_smooth: float = 2.0  # 大きいほど舵角変化を嫌う

	# 正規化用スケール
	clearance_norm: float = 2.0  # d[i_c] / clearance_norm で [0,1] に正規化

	# 安全距離 [mm]
	near_stop_dist: float = 500.0   # これ未満なら停止
	near_slow_dist: float = 1000.0   # これ未満なら減速
	near_back_dist: float = 200.0   # これ未満なら後退を検討

	# 進行方向を「直進」とみなす θ のデッドゾーン
	theta_dead_zone: float = 0.1  # |θ| <= 0.1 をほぼ直進とみなす

	# PWM 範囲
	pwm_center = 1500
	pwm_u_min = 1600 #1650以上
	pwm_l_min = 1400 #1350以下
	pwm_min = 1100
	pwm_max = 1900
	pwm_forward = 1750

class FTGIState(BaseState):
	"""FTG-i による深度ベース障害物回避状態."""

	def __init__(self):
		super().__init__()
		self._STATE = "ftgi"
		self.config = FTGIConfig()
		self._theta_prev: float = 0.0 # 前フレームの選択角度（舵角のスムーズ化に使用）
		self._frame_index: int = 0 # デバッグ用の連番フレームインデックス

	# -------------------------
	# BaseState 実装
	# -------------------------

	def needs_pose_estimation(self) -> bool:
		"""ArUco マーカーの姿勢推定は不要（depth のみ使用）"""
		return False

	def enter(self) -> None:
		rospy.loginfo("[FTGI] Entered -----------------")
		self._theta_prev = 0.0

	def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
		"""depth image から FTG-i を実行して ControlCommand を返す."""
		# logger = SessionLogger.get_instance()

		depth = camera_data.depth_image if camera_data is not None else None
		if depth is None:
			rospy.logwarn_throttle(1.0, "[FTGI] depth_image is None → STOP")
			self.log_event(event_type="depth_missing", data={}, frame_id=self._frame_index)
			return ControlCommand(CommandType.STOP)

		if depth.ndim != 2:
			rospy.logwarn_throttle(1.0, "[FTGI] depth_image is not 2D → STOP")
			return ControlCommand(CommandType.STOP)

		# --- 1D レンジ生成(d_1d: 1D 配列, min_dist: 最近距離, roi: 抽出した領域の depth 画像) ---
		d_1d, min_dist, roi = self._build_1d_range(depth)

		if d_1d is None or len(d_1d) == 0:
			rospy.logwarn_throttle(1.0, "[FTGI] failed to build 1D range → STOP")
			self.log_event(event_type="1d_range_failed", data={}, frame_id=self._frame_index)
			return ControlCommand(CommandType.STOP)
		
		# --- 画像ログ: ROI depth 画像を保存 ---
		if roi is not None:
			roi_vis = self._render_roi(roi)
			self.log_image(image=roi_vis, tag="roi_depth")
		
		# --- 画像ログ: 1D レンジヒートマップ ---
		heatmap_1d = self._render_1d_heatmap(d_1d)
		if heatmap_1d is not None:
			self.log_image(image=heatmap_1d, tag="1d_range")

		# --- ギャップ抽出 ---
		gaps = self._extract_gaps(d_1d)
		if not gaps:
			rospy.logwarn_throttle(0.5, "[FTGI] no free gaps → STOP")
			self.log_event(event_type="no_gaps", data={"min_dist": min_dist}, frame_id=self._frame_index)
			return ControlCommand(CommandType.STOP)

		# --- 進行方位選択（各ギャップ中心を候補方位とし, 評価関数 U(θ) を計算） ---
		theta_best, idx_best, gap_infos = self._select_best_heading(d_1d, gaps)
		self._theta_prev = theta_best

		# --- 画像ログ: ギャップ抽出＋方位選択 ---
		gap_vis = self._render_gap_debug(d_1d, gaps, gap_infos, theta_best, idx_best)
		if gap_vis is not None:
			self.log_image(image=gap_vis, tag="gap_select")

		# --- コマンド生成（方位に応じて ControlCommand を生成） ---
		cmd = self._heading_to_command(theta_best)
		print("cmd(ftgi_execute):", cmd)

		# --- メトリクスログ ---
		self.log_metric(
			metrics={
				"theta": theta_best,
				"min_dist": min_dist,
				"num_gaps": len(gaps),
				# "gap_width_pixels": gaps[idx_best][1] - gaps[idx_best][0],
			},
			tag="execute",
		)

		self._frame_index += 1
		return cmd

	def check_transition(
		self,
		imu_data: Optional[IMUData],
		camera_data: Optional[CameraData],
		markers: List[ArUcoMarker],
	) -> Optional[StateType]:
		"""状態遷移判定.
		マーカー検出．設定したIDが閾値距離内にあればマーカー追従状態へ遷移．
			- ここでは常に None を返して状態維持する（FTGI 状態からの遷移は別の状態で判定する想定）.
			- マーカーの距離確認は誤検知対策のため3フレーム連続で閾値内に入った場合遷移する
		"""
		# マーカー未検出
		if markers is None or len(markers) == 0:
			return None
		
		# マーカー検出あり → 遷移条件を満たすか確認		
		for m in markers:
			if m.marker_id in {20, 21} and m.distance is not None and m.distance < self.config.near_stop_dist:
				rospy.loginfo(f"[FTGI] Marker {m.marker_id} detected within {self.config.near_stop_dist}mm → transition to FOLLOW_ROW_CENTER")
				return StateType.FOLLOW_ROW_CENTER
			elif m.marker_id in {0, 10} and m.distance is not None and m.distance < self.config.near_stop_dist:
				rospy.loginfo(f"[FTGI] Marker {m.marker_id} detected within {self.config.near_stop_dist}mm → transition to APPROACH_MARKER")
				return StateType.APPROACH_MARKER
		return None

	def exit(self) -> None:
		rospy.loginfo("[FTGI] Exiting")

	# -------------------------
	# depth → 1D レンジ変換
	# -------------------------

	def _build_1d_range(
		self, depth: np.ndarray,
		min_percentile: float = 1.0 # 最小値を求める際に切る下位パーセンタイル（例: 1.0 → 下位1%を切る）
	) -> Tuple[Optional[np.ndarray], float, Optional[np.ndarray]]:
		"""depth image から 1D レンジ配列 d[i] を生成する.

		Returns:
			d_1d: shape=(K,) の 1D 距離[m] 配列 (NaN を含みうる)
			min_dist: 使用領域内の最近距離[m]（NaN は除外, 無い場合は inf）
			roi: 抽出した ROI 領域の depth 画像
		"""

		cfg = self.config
		h, w = depth.shape[:2]

		r0 = int(h * cfg.v_min_frac)
		r1 = int(h * cfg.v_max_frac)
		c0 = int(w * cfg.u_min_frac)
		c1 = int(w * cfg.u_max_frac)

		r0 = max(0, min(r0, h))
		r1 = max(0, min(r1, h))
		c0 = max(0, min(c0, w))
		c1 = max(0, min(c1, w))

		if r1 <= r0 or c1 <= c0:
			return None, float("inf"), None
		# ROI を抽出
		roi = depth[r0:r1, c0:c1]
		# 無効値を一括で NaN 化
		roi = np.where(np.isfinite(roi) & (roi > 0.0), roi, np.nan)

		# 最近距離を計算（1% トリムしてから最小値を取る）
		if np.any(np.isfinite(roi)):
			try:
				min_dist = float(np.nanpercentile(roi, min_percentile))
			except Exception:
				min_dist = float("inf")
		else:
			min_dist = float("inf")

		num_cols = roi.shape[1]
		K = min(cfg.num_samples, num_cols)
		sample_indices = np.linspace(0, num_cols - 1, K).astype(np.int32)

		# K列だけ抜く（shape: [H, K]）
		roi_s = roi[:, sample_indices]

		# 列ごとに中央値をとる（NaNは除外）
		d_1d = np.nanmedian(roi_s, axis=0)

		# --- 追加・変更・高速化-------

		# # 有効画素のみを使って最近距離を求める
		# valid_mask = np.isfinite(roi) & (roi > 0.0)
		# if np.any(valid_mask):
		# 	min_dist = float(np.min(roi[valid_mask]))
		# else:
		# 	min_dist = float("inf")

		# num_cols = roi.shape[1]
		# K = min(cfg.num_samples, num_cols)
		# if K <= 0:
		# 	return None, min_dist, roi

		# # 等間隔で K 本サンプルするインデックスを作成
		# sample_indices = np.linspace(0, num_cols - 1, K).astype(int)

		# d_list: List[float] = []
		# for idx in sample_indices:
		# 	col = roi[:, idx]
		# 	col_valid = col[np.isfinite(col) & (col > 0.0)]
		# 	if col_valid.size == 0:
		# 		d_list.append(np.nan)
		# 		continue

		# 	if cfg.use_trimmed_mean and col_valid.size > 0:
		# 		d_list.append(self._trimmed_mean(col_valid, cfg.trim_ratio))
		# 	else:
		# 		d_list.append(float(np.median(col_valid)))

		# d_1d = np.asarray(d_list, dtype=float)

		# # 1D メディアンフィルタ（NaN は無視）
		# ksize = max(1, cfg.median_filter_ksize)
		# if ksize > 1:
		# 	d_1d = self._nanmedian_filter_1d(d_1d, ksize)

		rospy.loginfo_throttle(
			1.0,
			f"[FTGI] ROI=({r0}:{r1},{c0}:{c1}) K={d_1d.size} "
			f"min={np.nanmin(d_1d):.2f} med={np.nanmedian(d_1d):.2f} max={np.nanmax(d_1d):.2f}"
		)
		return d_1d, min_dist, roi

	# def _save_roi_debug_image(self, roi: Optional[np.ndarray], theta: Optional[float], cmd: ControlCommand) -> None:
	# 	"""ROI depth 画像に情報を書き込み, ファイルとして保存する.

	# 	保存先: config.log_dir_roi_depth
	# 	ファイル名: 000000_CMD.png のような連番 + コマンド名.
	# 	"""

	# 	if roi is None:
	# 		return

	# 	try:
	# 		# depth[m] → 0-255 の 8bit グレースケールに正規化
	# 		roi_float = roi.astype(np.float32)
	# 		if not np.any(np.isfinite(roi_float) & (roi_float > 0.0)):
	# 			return
	# 		vis = cv2.normalize(roi_float, None, 0, 255, cv2.NORM_MINMAX)
	# 		vis = vis.astype(np.uint8)
	# 		# vis_color = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
	# 		vis_color = cv2.applyColorMap(vis, cv2.COLORMAP_JET)

	# 		# テキスト情報の組み立て
	# 		theta_str = "N/A" if theta is None else f"{theta:.3f}"
	# 		label = getattr(getattr(cmd, "command", None), "name", str(cmd.command))
	# 		pwm_val = None
	# 		if hasattr(cmd, "speed"):
	# 			spd = cmd.speed
	# 			if isinstance(spd, (int, float)):
	# 				pwm_val = int(spd)
	# 		pwm_str = "N/A" if pwm_val is None else str(pwm_val)

	# 		text1 = f"theta={theta_str}"
	# 		text2 = f"cmd={label} pwm={pwm_str}"
	# 		text3 = f"idx={self._frame_index}"

	# 		org1 = (10, 20)
	# 		org2 = (10, 40)
	# 		org3 = (10, 60)
	# 		font = cv2.FONT_HERSHEY_SIMPLEX
	# 		cv2.putText(vis_color, text1, org1, font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
	# 		cv2.putText(vis_color, text2, org2, font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
	# 		cv2.putText(vis_color, text3, org3, font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

	# 		# ファイル名: 000000_CMD.png
	# 		cmd_name_safe = label
	# 		filename = f"{self._frame_index:06d}_{cmd_name_safe}.png"
	# 		out_path = os.path.join(self.log_dir_roi_depth, filename)
	# 		cv2.imwrite(out_path, vis_color)
	# 	except Exception as e:
	# 		rospy.logwarn_throttle(1.0, f"[FTGI] failed to save ROI image: {e}")

	# def _save_1d_range_heatmap(self, d_1d: np.ndarray) -> None:
	# 	"""1D レンジ配列 d_1d をヒートマップとして保存する.

	# 	深い(距離が大きい)ほど色が強くなるように正規化し,
	# 	横軸=サンプル index, 縦軸=擬似的な高さの画像として保存する.
	# 	"""

	# 	if d_1d is None or d_1d.size == 0:
	# 		return

	# 	try:
	# 		vals = d_1d.astype(np.float32)
	# 		mask = np.isfinite(vals) & (vals > 0.0)
	# 		if not np.any(mask):
	# 			return

	# 		v_valid = vals[mask]
	# 		v_min = float(np.min(v_valid))
	# 		v_max = float(np.max(v_valid))
	# 		if v_max <= v_min:
	# 			v_max = v_min + 1e-6

	# 		norm = np.zeros_like(vals, dtype=np.float32)
	# 		norm[mask] = (vals[mask] - v_min) / (v_max - v_min)
	# 		norm = np.clip(norm, 0.0, 1.0)
	# 		row = (norm * 255.0).astype(np.uint8)[np.newaxis, :]

	# 		# 縦方向に引き伸ばしてヒートマップ画像にする
	# 		height = 80
	# 		img_gray = np.repeat(row, height, axis=0)
	# 		img_color = cv2.applyColorMap(img_gray, cv2.COLORMAP_JET)

	# 		filename = f"{self._frame_index:06d}_1d.png"
	# 		out_path = os.path.join(self.log_dir_1d_range, filename)
	# 		cv2.imwrite(out_path, img_color)
	# 	except Exception as e:
	# 		rospy.logwarn_throttle(1.0, f"[FTGI] failed to save 1D heatmap: {e}")

	@staticmethod
	def _trimmed_mean(values: np.ndarray, trim_ratio: float) -> float:
		"""values の 20% トリム平均などを計算（NaN は前処理済を想定）."""

		if values.size == 0:
			return float("nan")
		v_sorted = np.sort(values)
		k = int(len(v_sorted) * trim_ratio)
		if k > 0 and len(v_sorted) > 2 * k:
			v_sorted = v_sorted[k:-k]
		return float(np.mean(v_sorted))

	@staticmethod
	def _nanmedian_filter_1d(arr: np.ndarray, ksize: int) -> np.ndarray:
		"""NaN を無視する 1D メディアンフィルタ."""

		n = arr.size
		if n == 0 or ksize <= 1:
			return arr.copy()

		half = ksize // 2
		out = np.empty_like(arr)

		for i in range(n):
			s = max(0, i - half)
			e = min(n, i + half + 1)
			window = arr[s:e]
			valid = window[np.isfinite(window)]
			if valid.size == 0:
				out[i] = np.nan
			else:
				out[i] = np.median(valid)

		return out

	# -------------------------
	# ギャップ抽出 & 方位選択
	# -------------------------

	def _extract_gaps(self, d_1d: np.ndarray) -> List[Tuple[int, int, int]]:
		"""自由空間ギャップを抽出.

		Returns:
			gaps: List[(start_idx, end_idx, center_idx)] （end は inclusive）
		"""

		cfg = self.config

		# NaN は 0 (障害物) とみなす
		d_safe = np.nan_to_num(d_1d, nan=0.0, posinf=0.0, neginf=0.0)
		free = d_safe >= cfg.d_free
		# print(f"d_safe: {d_safe}, d_free={cfg.d_free}")  # デバッグ用出力

		K = d_safe.size
		min_width = max(cfg.min_gap_width_pixels, int(cfg.min_gap_width_ratio * K))

		gaps: List[Tuple[int, int, int]] = []
		i = 0
		while i < K:
			if not free[i]:
				i += 1
				continue
			start = i
			while i + 1 < K and free[i + 1]:
				i += 1
			end = i
			width = end - start + 1
			if width >= min_width:
				center = (start + end) // 2
				gaps.append((start, end, center))
			i += 1

		return gaps

	def _select_best_heading(
		self,
		d_1d: np.ndarray,
		gaps: List[Tuple[int, int, int]],
	) -> Tuple[float, int, List[dict]]:
		"""候補ギャップから評価関数 U(θ) を最大にする方位を選択.

		Returns:
			theta_best: 正規化方位 [-1,1]
			idx_best  : 1D 配列のインデックス
		"""

		cfg = self.config
		K = d_1d.size
		mid = (K - 1) / 2.0

		theta_best = 0.0
		idx_best = int(mid)
		u_best = -float("inf")
		gap_infos: List[dict] = []

		for (start, end, center) in gaps:
			d_center = d_1d[center]
			if not np.isfinite(d_center) or d_center <= 0.0:
				continue

			# θ を -1〜+1 に正規化
			theta = (center - mid) / max(mid, 1.0)

			# (A) Clearance 項
			clearance = max(0.0, min(d_center / cfg.clearance_norm, 1.0))

			# (B) Centering 項: 画像中心に近いほど良い
			center_dist = abs(center - mid) / max(mid, 1.0)
			centering = 1.0 - max(0.0, min(center_dist, 1.0))

			# (C) Goal heading 項（現状ゴール無しなので 0 か, 参考程度）
			yaw_goal = 0.0
			goal_diff = abs(theta - yaw_goal)
			goal_score = 1.0 - max(0.0, min(goal_diff, 1.0))

			# (D) Steering change penalty: 前回 θ_prev からの差分を嫌う
			steer_diff = abs(theta - self._theta_prev)
			steer_score = 1.0 - max(0.0, min(steer_diff, 1.0))

			u = (
				cfg.w_clearance * clearance
				+ cfg.w_centering * centering
				+ cfg.w_goal * goal_score
				+ cfg.w_steer_smooth * steer_score
			)

			gap_infos.append(
				{
					"start": start,
					"end": end,
					"center": center,
					"theta": float(theta),
					"clearance": float(clearance),
					"centering": float(centering),
					"goal": float(goal_score),
					"steer": float(steer_score),
					"u": float(u),
				}
			)

			if u > u_best:
				u_best = u
				theta_best = float(theta)
				idx_best = int(center)

		return theta_best, idx_best, gap_infos

	# def _save_gap_debug_image(
	# 	self,
	# 	d_1d: np.ndarray,
	# 	gaps: List[Tuple[int, int, int]],
	# 	gap_infos: List[dict],
	# 	theta_best: float,
	# 	idx_best: int,
	# ) -> None:
	# 	"""ギャップ抽出・方位選択結果を 1D ヒートマップ上に可視化して保存する.

	# 	- ベースは 1D レンジヒートマップ
	# 	- 各ギャップ区間を矩形で囲み, 評価値 U に応じて色を変える
	# 	- 最終的に選ばれたギャップは太線・白枠で強調
	# 	"""

	# 	if d_1d is None or d_1d.size == 0 or not gaps or not gap_infos:
	# 		return

	# 	try:
	# 		vals = d_1d.astype(np.float32)
	# 		mask = np.isfinite(vals) & (vals > 0.0)
	# 		if not np.any(mask):
	# 			return

	# 		v_valid = vals[mask]
	# 		v_min = float(np.min(v_valid))
	# 		v_max = float(np.max(v_valid))
	# 		if v_max <= v_min:
	# 			v_max = v_min + 1e-6

	# 		norm = np.zeros_like(vals, dtype=np.float32)
	# 		norm[mask] = (vals[mask] - v_min) / (v_max - v_min)
	# 		norm = np.clip(norm, 0.0, 1.0)
	# 		row = (norm * 255.0).astype(np.uint8)[np.newaxis, :]

	# 		height = 80
	# 		img_gray = np.repeat(row, height, axis=0)
	# 		img_color = cv2.applyColorMap(img_gray, cv2.COLORMAP_JET)

	# 		# U の min/max を取得
	# 		u_vals = np.array([info["u"] for info in gap_infos], dtype=np.float32)
	# 		u_min = float(np.min(u_vals))
	# 		u_max = float(np.max(u_vals))
	# 		if u_max <= u_min:
	# 			u_max = u_min + 1e-6

	# 		# 各ギャップを矩形で描画
	# 		for info in gap_infos:
	# 			start = int(info["start"])
	# 			end = int(info["end"])
	# 			center = int(info["center"])
	# 			u = float(info["u"])
	# 			u_norm = (u - u_min) / (u_max - u_min)
	# 			u_norm = max(0.0, min(u_norm, 1.0))

	# 			# U が高いほど明るい黄〜赤系になるように設定
	# 			color = (
	# 				int(255 * u_norm),              # B
	# 				int(255 * (1.0 - u_norm)),      # G
	# 				0,
	# 			)

	# 			top_left = (start, 0)
	# 			bottom_right = (end, height - 1)
	# 			cv2.rectangle(img_color, top_left, bottom_right, color, 1)

	# 			# U の簡易ラベルを中心付近に描く
	# 			label = f"U={u:.2f}"
	# 			org = (center, height // 2)
	# 			cv2.putText(img_color, label, org, cv2.FONT_HERSHEY_PLAIN, 0.7, color, 1, cv2.LINE_AA)

	# 		# 最終選択されたギャップを白枠で強調
	# 		for info in gap_infos:
	# 			start = int(info["start"])
	# 			end = int(info["end"])
	# 			center = int(info["center"])
	# 			if center == idx_best:
	# 				top_left = (start, 0)
	# 				bottom_right = (end, height - 1)
	# 				cv2.rectangle(img_color, top_left, bottom_right, (255, 255, 255), 2)
	# 				break

	# 		# 画像上部に θ_best など概要も追記
	# 		# text = f"theta_best={theta_best:.3f} idx_best={idx_best}"
	# 		# cv2.putText(img_color, text, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

	# 		filename = f"{self._frame_index:06d}_gap.png"
	# 		out_path = os.path.join(self.log_dir_gap, filename)
	# 		cv2.imwrite(out_path, img_color)
	# 	except Exception as e:
	# 		rospy.logwarn_throttle(1.0, f"[FTGI] failed to save gap debug image: {e}")

	# -------------------------
	# 安全チェック
	# -------------------------

	def _safety_command_if_needed(self, min_dist: float) -> Optional[ControlCommand]:
		"""近距離障害物に対する安全制御.

		min_dist: depth ROI 内の最近距離 [m]
		"""

		cfg = self.config
		print(f"[FTGI] min_dist={min_dist:.2f}m")  # デバッグ用出力

		if not np.isfinite(min_dist):
			return None

		if min_dist < cfg.near_back_dist:
			rospy.logwarn_throttle(0.5, f"[FTGI] obstacle very close ({min_dist:.2f}m) → BACKWARD")
			return ControlCommand(CommandType.MOVE_BACKWARD, cfg.pwm_backward)

		if min_dist < cfg.near_stop_dist:
			rospy.logwarn_throttle(0.5, f"[FTGI] obstacle close ({min_dist:.2f}m) → STOP")
			return ControlCommand(CommandType.STOP)

		if min_dist < cfg.near_slow_dist:
			# この場合, execute 側で進行方向を選んだうえで, 速度のみ抑えるようにしてもよいが
			# ここでは単純に「減速直進」とする
			rospy.loginfo_throttle(0.5, f"[FTGI] obstacle ahead ({min_dist:.2f}m) → SLOW FORWARD")
			return ControlCommand(CommandType.MOVE_FORWARD, cfg.pwm_forward_slow)

		return None

	# -------------------------
	# 方位 → ControlCommand 変換
	# -------------------------

	def _heading_to_command(self, theta: float, error_ref: float = 0.0) -> ControlCommand:
		error = max(-1.0, min(1.0, theta)) # クリップ　（左壁：正，右壁：負）
		e = error_ref - error # 誤差（左に壁なら負，右に壁なら正）

	    # デッドゾーン内は直進
		if abs(e) <= self.config.theta_dead_zone:
			return ControlCommand(CommandType.MOVE_FORWARD, self.config.pwm_forward)
		
		Kp = 250.0 # ゲイン（要調整）
		if e < 0: #左に壁，右旋回 (e < 0 なので Kp*e は負)
			pwm = int(self.config.pwm_u_min - Kp * e) # 右前進 pwm は 1500 以上になる
		else: #右に壁，左旋回（e > 0 なので Kp*e は正）
			pwm = int(self.config.pwm_l_min - Kp * e) # 左前進 pwm は 1500 以下になる
		pwm = max(self.config.pwm_min, min(self.config.pwm_max, pwm))  # クリップ

		if pwm < self.config.pwm_center:
			return ControlCommand(CommandType.FORWARD_LEFT, pwm)
		elif pwm > self.config.pwm_center:
			return ControlCommand(CommandType.FORWARD_RIGHT, pwm)


	# -------------------------
    # 描画ヘルパー（render_*）
    # -------------------------

	def _render_roi(self, roi: np.ndarray) -> np.ndarray:
		"""ROI depth 画像をカラーマップ変換して返す."""
		valid = np.isfinite(roi) & (roi > 0)
		if not np.any(valid):
			h, w = roi.shape[:2]
			return np.zeros((h, w, 3), dtype=np.uint8)

		v_min = float(np.min(roi[valid]))
		v_max = float(np.max(roi[valid]))
		if v_max <= v_min:
			v_max = v_min + 1e-6

		vis = np.zeros(roi.shape, dtype=np.float32)
		vis[valid] = (roi[valid] - v_min) / (v_max - v_min) * 255.0
		vis = np.clip(vis, 0, 255).astype(np.uint8)
		return cv2.applyColorMap(vis, cv2.COLORMAP_JET)

	def _render_roi_annotated(
        self,
        roi: np.ndarray,
        theta: float,
        cmd: ControlCommand,
        frame_id: int,
    ) -> np.ndarray:
		"""ROI depth 画像に θ・コマンド・フレーム番号を重ね描きして返す."""
		img = self._render_roi(roi)
		label = getattr(cmd.command, "name", str(cmd.command))
		pwm_str = str(int(cmd.speed)) if hasattr(cmd, "speed") and isinstance(cmd.speed, (int, float)) else "N/A"

		for y, text in enumerate([
			f"theta={theta:.3f}",
			f"cmd={label} pwm={pwm_str}",
			f"frame={frame_id}",
		], start=1):
			cv2.putText(
				img, text, (10, 20 * y),
				cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA,
			)
		return img

	def _render_1d_heatmap(
		self, d_1d: np.ndarray,
		d_min: float = 100, # [mm]
		d_max: float = 5000, # [mm]
		height: int = 80,   # ヒートマップの縦幅
	) -> Optional[np.ndarray]:
		"""1D レンジ配列をヒートマップ画像として返す

		横軸: サンプルインデックス (K)
		縦軸: 擬似的な高さ方向（同じ値を repeat）
		色:   距離が大きい（遠い）ほど暖色（JET カラーマップ）
		"""
		if d_1d is None or d_1d.size == 0:
			return None
		
		vals = d_1d.astype(np.float32)
		# --- 有効値マスク ---
		mask = np.isfinite(vals) & (vals > 0.0)
		if not np.any(mask):
			return None

		denom = float(d_max - d_min)
		if denom <= 0:
			raise ValueError("d_max must be greater than d_min")
		
		norm = np.zeros_like(vals, dtype=np.float32)
		norm[mask] = (vals[mask] - float(d_min)) / denom
		norm = np.clip(norm, 0.0, 1.0)

		# 1行 × K列 → 縦方向に repeat（直書き）
		row = (norm * 255.0).astype(np.uint8)[np.newaxis, :]  # サイズ: (1, K)
		img_gray = np.repeat(row, height, axis=0)                 # サイズ: (height, K)

		# JETカラーマップ適用
		img_color = cv2.applyColorMap(img_gray, cv2.COLORMAP_JET)

		return img_color

	def _render_gap_debug(
		self,
		d_1d: np.ndarray,
		gaps: List[Tuple[int, int, int]],
		gap_infos: List[dict],
		theta_best: float,
		idx_best: int,
	) -> Optional[np.ndarray]:
		"""
		ギャップ抽出・方位選択結果を 1D ヒートマップ上に描画して返す.

		- 各ギャップ区間を矩形で囲む（色: U 値に応じた青→赤グラデーション）
		- 選択されたギャップを白枠で強調
		- 各ギャップに U 値ラベルを描画
		"""
		img = self._render_1d_heatmap(d_1d)
		if img is None or not gap_infos:
			return img

		height = img.shape[0]
		u_vals = np.array([info["u"] for info in gap_infos], dtype=np.float32)
		u_min = float(np.min(u_vals))
		u_max = float(np.max(u_vals))
		if u_max <= u_min:
			u_max = u_min + 1e-6

		for info in gap_infos:
			start  = int(info["start"])
			end    = int(info["end"])
			center = int(info["center"])
			u_norm = float(np.clip((info["u"] - u_min) / (u_max - u_min), 0.0, 1.0))

			# U が高いほど赤，低いほど青
			color = (int(255 * u_norm), int(255 * (1.0 - u_norm)), 0)
			cv2.rectangle(img, (start, 0), (end, height - 1), color, 1)
			cv2.putText(
				img, f"U={info['u']:.2f}", (center, height // 2),
				cv2.FONT_HERSHEY_PLAIN, 0.7, color, 1, cv2.LINE_AA,
			)

		# 選択されたギャップを白枠で強調
		for info in gap_infos:
			if int(info["center"]) == idx_best:
				cv2.rectangle(
					img,
					(int(info["start"]), 0),
					(int(info["end"]), height - 1),
					(255, 255, 255), 2,
				)
				break

		# θ_best を画像上部に表示
		cv2.putText(
			img, f"theta_best={theta_best:.3f}", (5, 12),
			cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA,
		)
		return img

