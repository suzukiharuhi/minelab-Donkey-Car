from abc import ABC, abstractmethod
from typing import Optional, Callable


class BaseSensor(ABC):
    """センサ基底クラス"""
    
    def __init__(self):
        """
        センサー初期化
            - _callback: データ受信時のコールバック関数
            - _is_active: センサのアクティブ状態
        """
        self._callback: Optional[Callable] = None
        self._is_active = False
    
    def set_callback(self, callback: Callable):
        """データ受信時のコールバックを設定"""
        self._callback = callback
    
    @abstractmethod
    def start(self):
        """センサーの開始"""
        pass
    
    @abstractmethod
    def stop(self):
        """センサーの停止"""
        pass
    
    @property
    def is_active(self) -> bool:
        return self._is_active