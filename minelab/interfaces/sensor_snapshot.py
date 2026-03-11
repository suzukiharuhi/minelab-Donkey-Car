"""SensorSnapshot – convenience re-export from data_models.

Keeping a dedicated module makes ``from minelab.interfaces.sensor_snapshot
import SensorSnapshot`` work in addition to the combined import path.
"""
from minelab.interfaces.data_models import SensorSnapshot  # noqa: F401
