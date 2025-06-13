import signal
import threading

# 終了用の共有イベント
shutdown_event = threading.Event()

def signal_handler(sig, frame):
    print("\n[共通] Ctrl+C を検出。シャットダウンを開始します...")
    shutdown_event.set()

def register_signal_handler():
    signal.signal(signal.SIGINT, signal_handler)