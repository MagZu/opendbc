"""Tesla Pre-AP configuration: pedal calibration, radar, and control mode settings."""

import json
import os
import tempfile

from opendbc.car.carlog import carlog
from opendbc.car.tesla.preap.nap_params import NAPParamKeys

try:
  from openpilot.common.params import Params
  _params = Params()
  _PARAMS_AVAILABLE = True
except ImportError:
  _PARAMS_AVAILABLE = False

carlog.info("nap_conf: _PARAMS_AVAILABLE=%s", _PARAMS_AVAILABLE)


# ============================================
# Storage Configuration
# ============================================
CONFIG_FILE = "/data/nap_params.json"

# Default values for all parameters
DEFAULT_CONFIG = {
  # Control Modes
  'double_pull_window_ms': 750,  # 750ms - matches Tinkla's STALK_DOUBLE_PULL_MS
  # Longitudinal
  'use_pedal': False,
  'pedal_calibrated': False,
  'accel_profile': 'Chill',
  # Pedal Hardware
  'pedal_can_zero': False,
  'pedal_profile': 'P85+',
  # Pedal Calibration
  'pedal_min': 0,
  'pedal_max': 1023,
  'pedal_calib_min': -3.0,
  'pedal_calib_max': 99.6,
  'pedal_calib_zero': 0.0,
  'pedal_calib_factor': 1.0,
  # Radar
  'radar_enabled': False,
  'radar_behind_nosecone': False,
  'radar_offset': 0.0,
}


# ============================================
# Pedal DI (Driver Intent) Constants
# From Tinkla's tunes.py
# ============================================
# These are in "DI units" - the internal representation
# before calibration transform is applied
PEDAL_DI_MIN = -5       # Max regen braking (coasting hard)
PEDAL_DI_ZERO = 0       # Neutral (no accel, no regen)
PEDAL_DI_PRESSED = 2    # Threshold for "pedal pressed" detection

# Acceleration limits
ACCEL_MAX = 2.5         # m/s^2 - Max acceleration request
REGEN_MAX = -1.5        # m/s^2 - Max regen deceleration (speed dependent)

# Pedal hysteresis to prevent oscillation
PEDAL_HYST_GAP = 1.0    # Don't change pedal for oscillations within this range

# Speed-dependent max pedal values (from tunes.py)
# MPH:     0   11   27   44   67   90
# km/h:    0   18   43   72  108  144
PEDAL_BP = [0., 5., 12., 20., 30., 40.]  # m/s

# Pedal profiles: [S60, S85, P85, P85+, Generic]
PEDAL_PROFILES = {
  'S60':     [99., 99., 99., 99., 99., 99.],
  'S85':     [55., 63., 75., 90., 99., 99.],
  'P85':     [45., 52., 60., 67., 75., 82.],
  'P85+':    [37., 45., 52., 60., 67., 75.],
  'Generic': [99., 99., 99., 99., 99., 99.],
}

# Default to the most conservative profile for safer first-drive behavior
PEDAL_V_DEFAULT = PEDAL_PROFILES['P85+']

# Speed-dependent planner acceleration envelopes (from Tinkla tunes.py)
# Profiles: Chill (soft), Standard, MadMax (aggressive)
ACCEL_LOOKUP_BP = [0.0, 1.3, 7.5, 15.0, 25.0, 40.0]  # m/s
ACCEL_MAX_PROFILES = {
  'Chill': [0.3, 0.7, 0.9, 0.7, 0.6, 0.5],
  'Standard': [0.3, 0.9, 1.2, 1.0, 0.8, 0.6],
  'MadMax': [0.3, 1.6, 1.9, 1.5, 1.2, 1.0],
}
ACCEL_MAX_DEFAULT = ACCEL_MAX_PROFILES['Chill']

# Pedal profile index mapping (Params stores 1-4, nap_conf uses name strings)
_PROFILE_NAMES = list(PEDAL_PROFILES.keys())  # S60, S85, P85, P85+, Generic
_PROFILE_INDEX_TO_NAME = {i + 1: name for i, name in enumerate(_PROFILE_NAMES[:4])}
_PROFILE_NAME_TO_INDEX = {name: i for i, name in _PROFILE_INDEX_TO_NAME.items()}


def transform_di_to_pedal(val: float, pedal_zero: float, pedal_factor: float) -> float:
  """
  Convert DI (Driver Intent) units to actual pedal voltage.

  From Tinkla tunes.py:
    return PEDAL_ZERO + (val - PEDAL_DI_ZERO) / PEDAL_FACTOR

  Args:
    val: Value in DI units (-5 to ~100)
    pedal_zero: Calibrated zero point (voltage at coast)
    pedal_factor: Calibration factor from calibration tool

  Returns:
    Pedal voltage to send to Comma Pedal
  """
  if pedal_factor == 0:
    pedal_factor = 1.0  # Prevent division by zero
  return pedal_zero + (val - PEDAL_DI_ZERO) / pedal_factor


def transform_pedal_to_di(val: float, pedal_zero: float, pedal_factor: float) -> float:
  """
  Convert actual pedal voltage to DI (Driver Intent) units.

  From Tinkla tunes.py:
    return PEDAL_DI_ZERO + (val - PEDAL_ZERO) * PEDAL_FACTOR

  Args:
    val: Pedal voltage reading
    pedal_zero: Calibrated zero point (voltage at coast)
    pedal_factor: Calibration factor from calibration tool

  Returns:
    Value in DI units
  """
  return PEDAL_DI_ZERO + (val - pedal_zero) * pedal_factor


class NAPConf:
  """
  Configuration helper for Tesla Pre-AP vehicles.

  Uses a JSON file backend for persistent storage, avoiding
  OpenPilot's Params whitelist restrictions.

  Storage: /data/nap_params.json
  """

  def __init__(self):
    self._cache = {}
    self._load()

  # ============================================
  # Storage Backend (JSON File)
  # ============================================

  def _load(self) -> None:
    """Load configuration from JSON file, or use defaults."""
    try:
      if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, 'r') as f:
          loaded = json.load(f)
        # Merge with defaults (in case new keys were added)
        self._cache = {**DEFAULT_CONFIG, **loaded}
      else:
        # File doesn't exist - use defaults
        self._cache = DEFAULT_CONFIG.copy()
        self._save()  # Create the file with defaults
    except Exception:
      # JSON parse error or read error - use defaults
      self._cache = DEFAULT_CONFIG.copy()

  def _save(self) -> None:
    """
    Atomically save configuration to JSON file.

    Uses write-to-temp + rename pattern to prevent corruption
    if power cuts during write.
    """
    try:
      # Ensure /data directory exists
      os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)

      # Write to temporary file first
      fd, tmp_path = tempfile.mkstemp(
        dir=os.path.dirname(CONFIG_FILE),
        prefix='.nap_params_',
        suffix='.tmp'
      )
      try:
        with os.fdopen(fd, 'w') as f:
          json.dump(self._cache, f, indent=2)
        # Atomic rename
        os.replace(tmp_path, CONFIG_FILE)
      except Exception:
        # Clean up temp file on error
        try:
          os.unlink(tmp_path)
        except Exception:
          pass
        raise
    except Exception:
      pass  # Silently fail - don't crash the car

  def _get(self, key: str, default):
    """Get a value from cache with fallback to default."""
    return self._cache.get(key, default)

  def _put(self, key: str, value) -> None:
    """Set a value in cache and persist to disk."""
    self._cache[key] = value
    self._save()

  # ============================================
  # Params-backed Properties
  # Reads/writes openpilot Params when available,
  # falls back to JSON for standalone testing.
  # ============================================

  def _get_param_bool(self, param_key: str, json_key: str, default: bool = False) -> bool:
    if _PARAMS_AVAILABLE:
      return _params.get_bool(param_key)
    return self._get(json_key, default)

  def _put_param_bool(self, param_key: str, json_key: str, value: bool) -> None:
    if _PARAMS_AVAILABLE:
      _params.put_bool_nonblocking(param_key, bool(value))
    self._put(json_key, bool(value))

  def _get_param_float(self, param_key: str, json_key: str, default: float) -> float:
    if _PARAMS_AVAILABLE:
      val = _params.get(param_key, return_default=True)
      return float(val) if val is not None else default
    return float(self._get(json_key, default))

  def _put_param_float(self, param_key: str, json_key: str, value: float) -> None:
    if _PARAMS_AVAILABLE:
      _params.put(param_key, float(value))
    self._put(json_key, float(value))

  @property
  def use_pedal(self) -> bool:
    """True if Comma Pedal hardware is present and enabled."""
    return self._get_param_bool(NAPParamKeys.PEDAL_ENABLED, 'use_pedal')

  @use_pedal.setter
  def use_pedal(self, value: bool) -> None:
    self._put_param_bool(NAPParamKeys.PEDAL_ENABLED, 'use_pedal', value)

  @property
  def radar_enabled(self) -> bool:
    """True if using stock Bosch radar."""
    return self._get_param_bool(NAPParamKeys.RADAR_ENABLED, 'radar_enabled')

  @radar_enabled.setter
  def radar_enabled(self, value: bool) -> None:
    self._put_param_bool(NAPParamKeys.RADAR_ENABLED, 'radar_enabled', value)

  @property
  def radar_behind_nosecone(self) -> bool:
    """True if radar is behind the nosecone (signal attenuation adjustment)."""
    return self._get_param_bool(NAPParamKeys.RADAR_BEHIND_NOSECONE, 'radar_behind_nosecone')

  @radar_behind_nosecone.setter
  def radar_behind_nosecone(self, value: bool) -> None:
    self._put_param_bool(NAPParamKeys.RADAR_BEHIND_NOSECONE, 'radar_behind_nosecone', value)

  @property
  def pedal_calibrated(self) -> bool:
    """True if pedal has been calibrated."""
    return self._get_param_bool(NAPParamKeys.PEDAL_CALIB_DONE, 'pedal_calibrated')

  @pedal_calibrated.setter
  def pedal_calibrated(self, value: bool) -> None:
    self._put_param_bool(NAPParamKeys.PEDAL_CALIB_DONE, 'pedal_calibrated', value)

  @property
  def pedal_can_zero(self) -> bool:
    """True if pedal is on CAN bus 0 instead of bus 2."""
    if _PARAMS_AVAILABLE:
      bus = _params.get(NAPParamKeys.PEDAL_CAN_BUS, return_default=True)
      return bus == 0
    return self._get('pedal_can_zero', False)

  @pedal_can_zero.setter
  def pedal_can_zero(self, value: bool) -> None:
    if _PARAMS_AVAILABLE:
      _params.put(NAPParamKeys.PEDAL_CAN_BUS, 0 if value else 2)
    self._put('pedal_can_zero', bool(value))

  # ============================================
  # Double-Pull Engagement Parameters
  # ============================================

  @property
  def double_pull_enabled(self) -> bool:
    """
    Double-pull engagement mode is ALWAYS enabled for Pre-AP Tesla.

    This is a safety feature and cannot be disabled:
    - Single pull: Engage lateral control (steering) only
    - Double pull (within 750ms): Engage lateral + longitudinal
    """
    return True  # Always ON - not configurable for safety

  @property
  def double_pull_window_ms(self) -> int:
    """
    Time window in milliseconds to detect double-pull.
    Default: 750ms
    """
    return int(self._get('double_pull_window_ms', 750))

  @double_pull_window_ms.setter
  def double_pull_window_ms(self, value: int) -> None:
    # Clamp to 300 - 1500 ms
    self._put('double_pull_window_ms', max(300, min(1500, int(value))))

  @property
  def accel_profile(self) -> str:
    """
    Planner acceleration profile.
    - Chill: softer pickup and reduced low-speed aggression
    - Standard: Tinkla default balance
    - MadMax: maximum response
    """
    val = self._get('accel_profile', 'Chill')
    return val if val in ACCEL_MAX_PROFILES else 'Chill'

  @accel_profile.setter
  def accel_profile(self, value: str) -> None:
    if value in ACCEL_MAX_PROFILES:
      self._put('accel_profile', value)

  # ============================================
  # Pedal Calibration Parameters
  # ============================================

  @property
  def pedal_min(self) -> int:
    """Calibrated minimum pedal sensor value (released)."""
    return int(self._get('pedal_min', 0))

  @pedal_min.setter
  def pedal_min(self, value: int) -> None:
    self._put('pedal_min', int(value))

  @property
  def pedal_max(self) -> int:
    """Calibrated maximum pedal sensor value (floored)."""
    return int(self._get('pedal_max', 1023))

  @pedal_max.setter
  def pedal_max(self, value: int) -> None:
    self._put('pedal_max', int(value))

  @property
  def radar_offset(self) -> float:
    """Physical offset of the radar in meters."""
    return float(self._get('radar_offset', 0.0))

  @radar_offset.setter
  def radar_offset(self, value: float) -> None:
    self._put('radar_offset', float(value))

  @property
  def pedal_calib_min(self) -> float:
    """Calibrated minimum pedal value (from calibration tool)."""
    return self._get_param_float(NAPParamKeys.PEDAL_CALIB_MIN, 'pedal_calib_min', -3.0)

  @pedal_calib_min.setter
  def pedal_calib_min(self, value: float) -> None:
    self._put_param_float(NAPParamKeys.PEDAL_CALIB_MIN, 'pedal_calib_min', value)

  @property
  def pedal_calib_max(self) -> float:
    """Calibrated maximum pedal value (from calibration tool)."""
    return self._get_param_float(NAPParamKeys.PEDAL_CALIB_MAX, 'pedal_calib_max', 99.6)

  @pedal_calib_max.setter
  def pedal_calib_max(self, value: float) -> None:
    self._put_param_float(NAPParamKeys.PEDAL_CALIB_MAX, 'pedal_calib_max', value)

  @property
  def pedal_zero(self) -> float:
    """
    Calibrated pedal zero point (coast position).
    From Tinkla: PEDAL_ZERO = TeslaPedalCalibZero - 1 / PEDAL_FACTOR
    """
    if _PARAMS_AVAILABLE:
      calib_zero_val = _params.get(NAPParamKeys.PEDAL_CALIB_ZERO, return_default=True)
      calib_zero = float(calib_zero_val) if calib_zero_val is not None else 0.0
    else:
      calib_zero = float(self._get('pedal_calib_zero', 0.0))
    factor = self.pedal_factor
    if factor == 0:
      factor = 1.0
    return calib_zero - 1.0 / factor

  @pedal_zero.setter
  def pedal_zero(self, value: float) -> None:
    if _PARAMS_AVAILABLE:
      _params.put(NAPParamKeys.PEDAL_CALIB_ZERO, float(value))
    self._put('pedal_calib_zero', float(value))

  @property
  def pedal_factor(self) -> float:
    """
    Calibration factor: 100.0 / (pedal_max - pedal_pressed)
    Used to scale DI units to actual pedal voltage.
    """
    return self._get_param_float(NAPParamKeys.PEDAL_CALIB_FACTOR, 'pedal_calib_factor', 1.0)

  @pedal_factor.setter
  def pedal_factor(self, value: float) -> None:
    self._put_param_float(NAPParamKeys.PEDAL_CALIB_FACTOR, 'pedal_calib_factor', value)

  @property
  def pedal_profile(self) -> str:
    """Pedal profile name (S60, S85, P85, P85+, Generic)."""
    if _PARAMS_AVAILABLE:
      idx = _params.get(NAPParamKeys.PEDAL_PROFILE, return_default=True)
      return _PROFILE_INDEX_TO_NAME.get(idx, 'P85+')
    val = self._get('pedal_profile', 'P85+')
    return val if val in PEDAL_PROFILES else 'P85+'

  @pedal_profile.setter
  def pedal_profile(self, value: str) -> None:
    if value in PEDAL_PROFILES:
      if _PARAMS_AVAILABLE and value in _PROFILE_NAME_TO_INDEX:
        _params.put(NAPParamKeys.PEDAL_PROFILE, _PROFILE_NAME_TO_INDEX[value])
      self._put('pedal_profile', value)

  # ============================================
  # Utility Methods
  # ============================================

  @property
  def pedal_can_bus(self) -> int:
    """Returns the CAN bus number for the pedal (0 or 2)."""
    return 0 if self.pedal_can_zero else 2

  def get_pedal_profile_values(self) -> list:
    """Get the max pedal values for current profile."""
    return PEDAL_PROFILES.get(self.pedal_profile, PEDAL_V_DEFAULT)

  def get_accel_profile_values(self) -> list:
    """Get the planner accel envelope for current accel profile."""
    return ACCEL_MAX_PROFILES.get(self.accel_profile, ACCEL_MAX_DEFAULT)

  def di_to_pedal(self, val: float) -> float:
    """Convert DI units to pedal voltage using current calibration."""
    return transform_di_to_pedal(val, self.pedal_zero, self.pedal_factor)

  def pedal_to_di(self, val: float) -> float:
    """Convert pedal voltage to DI units using current calibration."""
    return transform_pedal_to_di(val, self.pedal_zero, self.pedal_factor)

  def print_config(self) -> None:
    """Print current configuration to console."""
    print("=== Tesla Pre-AP Configuration ===")
    storage = "openpilot Params" if _PARAMS_AVAILABLE else CONFIG_FILE
    print(f"    Storage: {storage}")
    print("")
    print("  [CONTROL MODES]")
    print(f"    Double-Pull Mode:     ON (always enabled)")
    print("")
    print("  [LONGITUDINAL]")
    print(f"    Pedal Enabled:        {'ON' if self.use_pedal else 'OFF'}")
    print(f"    Pedal Calibrated:     {'YES' if self.pedal_calibrated else 'NO'}")
    print(f"    Accel Profile:        {self.accel_profile}")
    print("")
    print("  [PEDAL CALIBRATION]")
    print(f"    Pedal Min (raw):      {self.pedal_min}")
    print(f"    Pedal Max (raw):      {self.pedal_max}")
    print(f"    Pedal Calib Min:      {self.pedal_calib_min:.2f}")
    print(f"    Pedal Calib Max:      {self.pedal_calib_max:.2f}")
    print(f"    Pedal Zero:           {self.pedal_zero:.3f}")
    print(f"    Pedal Factor:         {self.pedal_factor:.3f}")
    print(f"    Pedal Profile:        {self.pedal_profile}")
    print(f"    Pedal CAN Bus:        {self.pedal_can_bus}")
    print("")
    print("  [RADAR]")
    print(f"    Radar Enabled:        {'ON' if self.radar_enabled else 'OFF'}")
    print(f"    Behind Nosecone:      {'YES' if self.radar_behind_nosecone else 'NO'}")
    print(f"    Radar Offset:         {self.radar_offset}m")
    print("")
    print("==================================")

  def get_all_params(self) -> dict:
    """Get all parameters as a dictionary (for CLI tool)."""
    return {
      # Control Modes (double_pull is always ON, not configurable)
      'double_pull_window_ms': self.double_pull_window_ms,
      # Longitudinal
      'use_pedal': self.use_pedal,
      'pedal_calibrated': self.pedal_calibrated,
      'accel_profile': self.accel_profile,
      # Pedal Calibration
      'pedal_min': self.pedal_min,
      'pedal_max': self.pedal_max,
      'pedal_calib_min': self.pedal_calib_min,
      'pedal_calib_max': self.pedal_calib_max,
      'pedal_zero': self.pedal_zero,
      'pedal_factor': self.pedal_factor,
      'pedal_profile': self.pedal_profile,
      'pedal_can_bus': self.pedal_can_bus,
      'pedal_can_zero': self.pedal_can_zero,
      # Radar
      'radar_enabled': self.radar_enabled,
      'radar_behind_nosecone': self.radar_behind_nosecone,
      'radar_offset': self.radar_offset,
    }

  def reset_to_defaults(self) -> None:
    """Reset all parameters to safe defaults."""
    self._cache = DEFAULT_CONFIG.copy()
    self._save()

  def reload(self) -> None:
    """Reload configuration from disk (useful for external edits)."""
    self._load()


# Singleton instance for easy import
nap_conf = NAPConf()
