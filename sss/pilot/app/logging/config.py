from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class LoggingConfig:
    """ログ設定"""
    # ログ保存ルートディレクトリ
    log_root: Path = Path("/home/pi/ugv_ws/logging")

    # 画像保存設定
    save_images: bool = True
    image_format: str = "jpg"           # "jpg" or "png"  (jpg の方が高速)
    image_quality: int = 85             # JPEG 品質 (1-100) ※png では無視

    # 非同期画像書き込みワーカー設定
    async_writer_workers: int = 2       # ワーカースレッド数 (RGB+Depth で 2 が最適)
    async_writer_queue_size: int = 60   # キュー上限フレーム数 (溢れたら最古を破棄)

    # CSV設定
    save_csv: bool = True
    csv_flush_interval: int = 10        # 何行ごとにflushするか

    # フレーム番号のゼロ埋め桁数
    frame_digits: int = 6

    # 詳細デバッグ保存（重い処理）
    verbose: bool = False