"""
非同期画像書き込みワーカー．

Producer-Consumer パターンで画像保存をメインループから切り離す．
  Producer (メインループ) : enqueue() でキューに画像を積む → ほぼ0ms
  Consumer (ワーカースレッド): queue から取り出して cv2.imwrite() を実行

Usage:
    writer = AsyncImageWriter(num_workers=2, max_queue_size=60)
    writer.start()

    writer.enqueue(Path("images/raw/000001_rgb.jpg"), rgb_array)
    writer.enqueue(Path("images/raw/000001_depth.png"), depth_array)

    writer.stop()   # キューを全部処理してから終了
"""

import queue
import threading
from pathlib import Path
from typing import Callable, Optional, Tuple, Union

import cv2
import numpy as np

try:
    import rospy
    _USE_ROSPY = True
except ImportError:
    import logging as _logging
    _USE_ROSPY = False


def _log_info(msg: str) -> None:
    if _USE_ROSPY:
        rospy.loginfo(msg)
    else:
        print("[INFO]", msg)


def _log_warn(msg: str) -> None:
    if _USE_ROSPY:
        rospy.logwarn_throttle(5.0, msg)
    else:
        print("[WARN]", msg)


# (保存パス, 画像データ or 遅延レンダリング関数, JPEGパラメータ or None)
_Item = Tuple[Path, Union[np.ndarray, Callable[[], Optional[np.ndarray]]], Optional[list]]


class AsyncImageWriter:
    """
    画像を非同期でディスクへ書き込むワーカープール．

    スレッドセーフ: enqueue() / stop() は任意のスレッドから呼べる．
    """

    def __init__(self, num_workers: int = 2, max_queue_size: int = 60):
        """
        Args:
            num_workers:    ワーカースレッド数（RGB+Depth の同時書き込みには 2 が最適）．
            max_queue_size: キューの上限．溢れた場合は最古フレームを破棄して警告を出す．
        """
        self._queue: queue.Queue[Optional[_Item]] = queue.Queue(maxsize=max_queue_size)
        self._num_workers = num_workers
        self._workers: list = []
        self._saved_count = 0
        self._dropped_count = 0
        self._lock = threading.Lock()
        self._running = False

    # ------------------------------------------------------------------
    # ライフサイクル
    # ------------------------------------------------------------------

    def start(self) -> None:
        """ワーカースレッドを起動する．"""
        if self._running:
            return
        self._running = True
        for i in range(self._num_workers):
            t = threading.Thread(
                target=self._worker_loop,
                name=f"AsyncImageWriter-{i}",
                daemon=True,
            )
            t.start()
            self._workers.append(t)
        _log_info(
            f"[AsyncImageWriter] started: workers={self._num_workers} "
            f"queue_max={self._queue.maxsize}"
        )

    def stop(self, timeout: float = 10.0) -> None:
        """
        ワーカーを停止する．

        キューに積まれた残り全フレームを書き出してから終了する(drain)．
        timeout 秒以内に完了しない場合は強制終了する．
        """
        if not self._running:
            return
        self._running = False
        # 終了シグナル（None）をワーカー数分投入
        for _ in self._workers:
            self._queue.put(None)
        for t in self._workers:
            t.join(timeout=timeout)
        _log_info(
            f"[AsyncImageWriter] stopped: "
            f"saved={self._saved_count} dropped={self._dropped_count} "
            f"queue_remaining={self._queue.qsize()}"
        )
        self._workers.clear()

    # ------------------------------------------------------------------
    # Producer API
    # ------------------------------------------------------------------

    def enqueue(
        self,
        path: Path,
        image: np.ndarray,
        jpeg_params: Optional[list] = None,
    ) -> bool:
        """
        画像をキューに積む（ノンブロッキング）．

        キューが満杯の場合: 最古エントリを1つ破棄して再積みし False を返す．

        Args:
            path:         保存先フルパス．親ディレクトリが存在しない場合は自動作成する．
            image:        numpy 画像配列．内部で copy() するため呼び出し元の変更に影響されない．
            jpeg_params:  cv2.imwrite に渡すパラメータリスト（例: [cv2.IMWRITE_JPEG_QUALITY, 85]）．
                          None の場合はデフォルト設定で保存．

        Returns:
            True  : 通常エンキュー成功
            False : キュー満杯によりドロップ発生
        """
        item: _Item = (path, image.copy(), jpeg_params)
        return self._put(item)

    def enqueue_lazy(
        self,
        path: Path,
        render_fn: Callable[[], Optional[np.ndarray]],
        jpeg_params: Optional[list] = None,
    ) -> bool:
        """
        描画関数をキューに積む（ノンブロッキング）．
        描画（cv2処理）はワーカースレッドで実行されるため，
        メインループへの負荷はほぼゼロ．

        Args:
            path:        保存先フルパス．
            render_fn:   引数なしで np.ndarray を返す描画関数．None を返した場合は保存しない．
                         ※ numpy 配列を参照する場合は呼び出し元で copy() してクロージャに渡すこと．
            jpeg_params: cv2.imwrite に渡すパラメータ（jpg のみ有効）．
        """
        item: _Item = (path, render_fn, jpeg_params)
        return self._put(item)

    def enqueue_bundle(
        self,
        paths: list,
        render_fn: Callable[[], Optional[list]],
        jpeg_params: Optional[list] = None,
    ) -> bool:
        """
        1 回の render_fn 呼び出しで複数ファイルを保存する．

        render_fn は np.ndarray のリストを返す関数．
        paths[i] に対して返されたリストの [i] 番目の画像を書き出す．
        None を返した要素は書き出しをスキップ．

        メインループへの影響: ~0.01ms (queue.put のみ)

        Args:
            paths:      保存先パスのリスト（render_fn の戻り値と同数）
            render_fn:  () -> List[Optional[np.ndarray]] を返す描画関数
            jpeg_params: cv2.imwrite パラメータ（全ファイル共通）
        """
        params = jpeg_params or []
        lock_ref = self._lock
        saved_ref = self

        def _bundle(paths=paths, render_fn=render_fn, params=params):
            images = render_fn()
            if images is None:
                return None  # _write() が None チェックで早期リターン
            for path, img in zip(paths, images):
                if img is None:
                    continue
                try:
                    path.parent.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(path), img, params)
                    with lock_ref:
                        saved_ref._saved_count += 1
                except Exception as e:
                    _log_warn(f"[AsyncImageWriter] bundle write error ({path}): {e}")
            return None  # 書き込み済み → 外側の _write() は no-op

        # sentinel として先頭パスを渡す（_write(None) で早期リターンするため実際は未使用）
        return self._put((paths[0], _bundle, None))

    # ------------------------------------------------------------------
    # 統計
    # ------------------------------------------------------------------

    @property
    def queue_size(self) -> int:
        """現在のキュー積み残し数."""
        return self._queue.qsize()

    @property
    def saved_count(self) -> int:
        return self._saved_count

    @property
    def dropped_count(self) -> int:
        return self._dropped_count

    # ------------------------------------------------------------------
    # 内部ヘルパー
    # ------------------------------------------------------------------

    def _put(self, item: "_Item") -> bool:
        """キューに積む共通ロジック（ノンブロッキング）．満杯なら最古を破棄して再投入．"""
        try:
            self._queue.put_nowait(item)
            return True
        except queue.Full:
            try:
                self._queue.get_nowait()
                with self._lock:
                    self._dropped_count += 1
            except queue.Empty:
                pass
            _log_warn(
                f"[AsyncImageWriter] queue full — oldest frame dropped "
                f"(dropped_total={self._dropped_count})"
            )
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                pass
            return False

    # ------------------------------------------------------------------
    # Consumer (ワーカースレッド)
    # ------------------------------------------------------------------

    def _worker_loop(self) -> None:
        """ワーカースレッドのメインループ．None を受け取ったら終了．"""
        while True:
            item = self._queue.get()
            if item is None:            # 終了シグナル
                self._queue.task_done()
                break
            path, payload, jpeg_params = item
            # 遅延レンダリング: 描画関数をここで実行する
            image = payload() if callable(payload) else payload
            self._write(path, image, jpeg_params)
            self._queue.task_done()

    def _write(self, path: Path, image: np.ndarray, jpeg_params: Optional[list]) -> None:
        """実際にディスクへ書き込む．"""
        if image is None:
            return
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            params = jpeg_params or []
            success = cv2.imwrite(str(path), image, params)
            if success:
                with self._lock:
                    self._saved_count += 1
            else:
                _log_warn(f"[AsyncImageWriter] cv2.imwrite failed: {path}")
        except Exception as e:
            _log_warn(f"[AsyncImageWriter] write error ({path}): {e}")
