"""Lightweight signal filters (ROS-free).

All filters operate on plain Python floats or numpy arrays.
"""
from __future__ import annotations

from typing import Optional


class ExponentialMovingAverage:
    """Single-value EMA filter.

    Args:
        alpha:         Smoothing factor in (0, 1].  Higher = more reactive.
        initial_value: Seed value.  If ``None``, the first sample is used.
    """

    def __init__(
        self, alpha: float = 0.2, initial_value: Optional[float] = None
    ) -> None:
        if not (0.0 < alpha <= 1.0):
            raise ValueError(f"alpha must be in (0, 1], got {alpha}")
        self._alpha = alpha
        self._value: Optional[float] = initial_value

    def update(self, sample: float) -> float:
        """Update the filter with *sample* and return the smoothed value."""
        if self._value is None:
            self._value = sample
        else:
            self._value = (1.0 - self._alpha) * self._value + self._alpha * sample
        return self._value

    @property
    def value(self) -> Optional[float]:
        return self._value

    def reset(self) -> None:
        self._value = None
