"""
Shim module to support legacy imports while the real implementations live in
the package directory `states/auto_charging/`.

This file exposes the subpackage classes and enables importing submodules like
`pilot.app.states.auto_charging.adjust_position_state` even though this file
shares the same name as the package directory.
"""
import os
import importlib

# Treat this module as a package by setting __path__ to the real directory
__path__ = [os.path.join(os.path.dirname(__file__), 'auto_charging')]

# Re-export classes for `from pilot.app.states.auto_charging import X` style
AutoChargingState = importlib.import_module(__name__ + '.auto_charging_state').AutoChargingState
ApproachEntryPointState = importlib.import_module(__name__ + '.approach_entry_point_state').ApproachEntryPointState
AdjustPositionState = importlib.import_module(__name__ + '.adjust_position_state').AdjustPositionState
DockingState = importlib.import_module(__name__ + '.docking_state').DockingState
ChargingState = importlib.import_module(__name__ + '.charging_state').ChargingState
RetryingState = importlib.import_module(__name__ + '.retrying_state').RetryingState

__all__ = [
    'AutoChargingState',
    'ApproachEntryPointState',
    'AdjustPositionState',
    'DockingState',
    'ChargingState',
    'RetryingState',
]
