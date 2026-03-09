"""
Pre-AP Comma Pedal feedback parser.

Extracts GAS_SENSOR (0x552) CAN parsing and pedal health tracking from carstate.py.
Ported from Tinkla's carstate.py pedal interceptor parsing.
"""
from opendbc.car.tesla.nap_conf import nap_conf, PEDAL_DI_PRESSED

PEDAL_TIMEOUT_MS = 500


class PedalFeedback:
  """
  Parses Comma Pedal GAS_SENSOR feedback and tracks pedal health.

  Attributes read by carcontroller (via CarState bridge):
    interceptor_value, timeout, available
  """

  def __init__(self):
    self.interceptor_value = 0.0
    self.interceptor_value2 = 0.0
    self.interceptor_state = 0
    self.idx = 0
    self.prev_idx = 0
    self.last_seen_ms = 0
    self.available = False
    self.timeout = True
    self.torque_level = 0.0

  def update(self, gas_sensor_msg, curr_time_ms):
    """Parse GAS_SENSOR message and update pedal health state.

    Args:
      gas_sensor_msg: dict from cp_ap_party.vl.get("GAS_SENSOR", {}), may be empty
      curr_time_ms: current time in milliseconds

    Returns:
      True if message was parsed, False if empty/failed.
    """
    try:
      if not gas_sensor_msg:
        return False

      self.prev_idx = self.idx

      # Read pedal sensor values
      # From DBC: INTERCEPTOR_GAS, INTERCEPTOR_GAS2, STATE, IDX
      interceptor_gas = float(gas_sensor_msg.get("INTERCEPTOR_GAS", 0.0))
      interceptor_gas2 = float(gas_sensor_msg.get("INTERCEPTOR_GAS2", 0.0))
      self.interceptor_state = int(gas_sensor_msg.get("STATE", 0))
      self.idx = int(gas_sensor_msg.get("IDX", 0))

      # Convert decoded pedal value to DI units.
      # Do NOT apply M1/M2 scaling here; DBC decoding already did that.
      self.interceptor_value = float(nap_conf.pedal_to_di(interceptor_gas))
      self.interceptor_value2 = float(nap_conf.pedal_to_di(interceptor_gas2))

      # Track pedal responsiveness
      if self.idx != self.prev_idx:
        self.last_seen_ms = curr_time_ms

      # Check pedal timeout (500ms without message)
      self.timeout = (curr_time_ms - self.last_seen_ms) > PEDAL_TIMEOUT_MS
      self.available = (not self.timeout) and (self.interceptor_state == 0)
      return True

    except Exception:
      # Pedal not present or parsing failed
      self.available = False
      self.timeout = True
      return False

  def update_torque(self, di_torque1_msg):
    """Read torque level from DI_torque1 for pedal zero learning."""
    try:
      self.torque_level = di_torque1_msg.get("DI_torqueMotor", 0)
    except Exception:
      self.torque_level = 0.0

  @property
  def gas_pressed(self):
    """True if pedal interceptor value exceeds press threshold."""
    return self.interceptor_value > PEDAL_DI_PRESSED
