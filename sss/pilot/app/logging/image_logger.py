from pathlib import Path
from typing import Callable, Optional

import cv2
import numpy as np

from .async_image_writer import AsyncImageWriter
from .config import LoggingConfig


class ImageLogger:
    """
    画像の保存を担当する.

    内部で AsyncImageWriter を使い，cv2.imwrite をワーカースレッドへ委譲する．
    メインループへの影響は enqueue() のコスト（~0.1ms）のみ．
    """

    def __init__(self, session_dir: Path, config: LoggingConfig):
        self._session_dir = session_dir
        self._config = config
        self._images_dir = session_dir / "images"

        self._writer = AsyncImageWriter(
            num_workers=config.async_writer_workers,
            max_queue_size=config.async_writer_queue_size,
        )
        self._writer.start()

    # ------------------------------------------------------------------
    # 保存 API
    # ------------------------------------------------------------------

    def save(
        self,
        frame_id: int,
        image: np.ndarray,
        tag: str,
        state: str = "",
        subdir: Optional[str] = None,
    ) -> Path:
        """
        画像をキューに積む（ノンブロッキング）．

        ファイル名: {frame_id:06d}_{tag}.{ext}
        保存先:    images/{subdir or state}/
        """
        if image is None:
            return None

        sub = subdir or state or "misc"
        ext = self._config.image_format
        fname = f"{str(frame_id).zfill(self._config.frame_digits)}_{tag}.{ext}"
        fpath = self._images_dir / sub / fname

        jpeg_params = None
        if ext == "jpg":
            jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, self._config.image_quality]

        self._writer.enqueue(fpath, image, jpeg_params)
        return fpath

    def save_lazy(
        self,
        frame_id: int,
        render_fn: Callable[[], Optional[np.ndarray]],
        tag: str = "",
        state: str = "",
        subdir: Optional[str] = None,
    ) -> Path:
        """
        描画関数をキューに積む（ノンブロッキング）．
        描画はワーカースレッドで実行されるため，メインループへの負荷はほぼゼロ．

        ※ render_fn 内で numpy 配列を参照する場合は，
          呼び出し元で .copy() しデフォルト引数でキャプチャすること．
          例: save_lazy(frame_id, lambda d=depth.copy(): render(d), ...)
        """
        sub = subdir or state or "misc"
        ext = self._config.image_format
        fname = f"{str(frame_id).zfill(self._config.frame_digits)}_{tag}.{ext}"
        fpath = self._images_dir / sub / fname

        jpeg_params = None
        if ext == "jpg":
            jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, self._config.image_quality]

        self._writer.enqueue_lazy(fpath, render_fn, jpeg_params)
        return fpath

    def save_lazy_bundle(
        self,
        frame_id: int,
        render_fn: Callable[[], Optional[list]],
        tags: list,
        state: str = "",
        subdir: Optional[str] = None,
    ) -> list:
        """
        1 回の render_fn 呼び出しで複数タグの画像をまとめて保存する．
        render_fn は len(tags) 個の np.ndarray（or None）のリストを返すこと．
        """
        sub = subdir or state or "misc"
        ext = self._config.image_format
        jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, self._config.image_quality] if ext == "jpg" else None

        paths = [
            self._images_dir / sub / f"{str(frame_id).zfill(self._config.frame_digits)}_{tag}.{ext}"
            for tag in tags
        ]
        self._writer.enqueue_bundle(paths, render_fn, jpeg_params)
        return paths

    # ------------------------------------------------------------------
    # ライフサイクル
    # ------------------------------------------------------------------

    def close(self, drain_timeout: float = 10.0) -> None:
        """ワーカーを停止する．キューの残り全フレームを書き出してから終了する．"""
        self._writer.stop(timeout=drain_timeout)

    # ------------------------------------------------------------------
    # 統計
    # ------------------------------------------------------------------

    @property
    def saved_count(self) -> int:
        return self._writer.saved_count

    @property
    def dropped_count(self) -> int:
        return self._writer.dropped_count

    @property
    def queue_size(self) -> int:
        return self._writer.queue_size