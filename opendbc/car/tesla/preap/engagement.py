"""
Pre-AP engagement state machine (double-pull stalk, cruise buttons, brake override).

Ported from Tinkla's PCC_module.py and LONG_module.py engagement patterns.
Extracted from carstate.py to isolate pre-AP logic from AP1+ code paths.
"""
from opendbc.car import structs
from opendbc.car.carlog import carlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CruiseButtons

ButtonType = structs.CarState.ButtonEvent.Type


class PreAPEngagement:
  """
  Manages pre-AP engagement state: double-pull detection, button events,
  target speed, brake override, and CC spoof flags.

  Attributes read by carcontroller (via CarState bridge):
    cruiseEnabled, enableLongControl, enableJustCC, pedal_speed_kph,
    longCtrlEvent, preap_cc_cancel_needed, preap_cc_engage_needed
  """

  def __init__(self, double_pull_enabled, double_pull_window_ms):
    self.enableDoublePull = double_pull_enabled
    self.double_pull_window_ms = double_pull_window_ms

    self.cruiseEnabled = False
    self.enableLongControl = False
    self.enableJustCC = False
    self.pending_enable = False

    # Double-pull timing
    self.stalk_pull_time_ms = 0
    self.prev_stalk_pull_time_ms = -1000  # Start negative to avoid false double-pull on first press

    # Software-managed target speed (Tinkla PCC_module port)
    # Pre-AP has no stock cruise, so we manage the target speed ourselves.
    self.pedal_speed_kph = 0.0

    # Alert event: "pccEnabled", "pccDisabled", etc.
    self.longCtrlEvent = None

    # CC spoof flags (read by carcontroller)
    self.preap_cc_cancel_needed = False
    self.preap_cc_engage_needed = False
    self.preap_last_cc_spoof_ms = 0

    # Brake override tracking
    self.preap_brake_pressed_prev = False

    # Echo filter: timestamp of last MAIN/RES/DECEL press edge
    self.last_stalk_non_cancel_ms = -10000

    # Steering disengage edge detector
    self.prev_steering_disengage = False

  def handle_steering_disengage(self, steering_disengage):
    """Reset engagement on steering disengage rising edge.

    This mirrors panda safety behavior (controls_allowed is dropped on the same edge)
    and guarantees the next engagement comes from a fresh stalk pull sequence.
    """
    if steering_disengage and not self.prev_steering_disengage:
      was_long_active = self.enableLongControl
      self.cruiseEnabled = False
      self.enableLongControl = False
      self.enableJustCC = False
      self.pending_enable = False
      self.pedal_speed_kph = 0.0
      self.stalk_pull_time_ms = 0
      self.prev_stalk_pull_time_ms = -1000
      if was_long_active:
        self.longCtrlEvent = "pccDisabled"
    self.prev_steering_disengage = steering_disengage

  def process_buttons(self, cruise_buttons, prev_cruise_buttons, curr_time_ms,
                      v_ego, speed_units, use_pedal, pedal_long_allowed,
                      long_control_allowed, real_brake_pressed):
    """Process stalk inputs. Updates internal state. Returns list of ButtonEvents.

    Args:
      cruise_buttons: Current STW_ACTN_RQ.SpdCtrlLvr_Stat value
      prev_cruise_buttons: Previous frame's value
      curr_time_ms: Current time in milliseconds
      v_ego: Vehicle speed in m/s
      speed_units: "MPH" or "KPH" from DI_state
      use_pedal: True if Comma Pedal hardware is enabled
      pedal_long_allowed: True if pedal can actuate (use_pedal AND valid transform)
      long_control_allowed: True if any longitudinal source is available
      real_brake_pressed: True if driver is pressing brake pedal
    """
    button_events = []

    # ==============================================
    # MAIN button: Rising edge detection (Tinkla pattern)
    # From Tinkla's PCC_module.py lines 152-161:
    #   if (CS.cruise_buttons == CruiseButtons.MAIN
    #       and self.prev_cruise_buttons != CruiseButtons.MAIN):
    # This ONLY fires when button BECOMES MAIN, not on release.
    # ==============================================
    if (cruise_buttons == CruiseButtons.MAIN
        and prev_cruise_buttons != CruiseButtons.MAIN):
      carlog.warning("STALK MAIN pull detected | "
                     "cruiseEnabled=%s enableLong=%s enableJustCC=%s pending=%s "
                     "use_pedal=%s long_allowed=%s doublePull=%s",
                     self.cruiseEnabled, self.enableLongControl,
                     self.enableJustCC, self.pending_enable,
                     use_pedal, long_control_allowed, self.enableDoublePull)
      # On Pre-AP, EAC_INHIBITED is the normal EPS idle state (no AP ECU present).
      # The EPS transitions INHIBITED -> AVAILABLE -> ACTIVE once it sees valid
      # EPAS steer commands from the carcontroller.  Tinkla never gated engagement
      # on steerFaultTemporary — it let the system engage and suppressed lateral
      # torque (latActive=False) until the EPS cleared.  We match that behavior.
      if self.enableDoublePull:
        self._handle_double_pull(curr_time_ms, v_ego, speed_units,
                                 use_pedal, pedal_long_allowed, long_control_allowed)
      else:
        # Double-pull disabled: single pull = full control (pedal or stock cruise).
        carlog.warning("STALK single-pull engage (doublePull disabled) — full control")
        self.cruiseEnabled = True
        self.pending_enable = False
        self.enableLongControl = long_control_allowed
        self.enableJustCC = not long_control_allowed
        if pedal_long_allowed:
          self.pedal_speed_kph = self._capture_target_speed(v_ego, speed_units)
        else:
          self.pedal_speed_kph = 0.0
          if not use_pedal:
            self.preap_cc_engage_needed = True
            self.preap_last_cc_spoof_ms = curr_time_ms

    # General button event handling (for UI/buttonEvents)
    if cruise_buttons != prev_cruise_buttons:
      be = self._make_button_event(cruise_buttons, prev_cruise_buttons, curr_time_ms,
                                   v_ego, speed_units, use_pedal)
      button_events.append(be)

    # Double-pull window expired — lateral is already engaged from the first
    # pull, so just clear the pending flag.
    if self.pending_enable:
      time_since_pull = curr_time_ms - self.stalk_pull_time_ms
      if time_since_pull > self.double_pull_window_ms:
        self.pending_enable = False

    # Brake press drops longitudinal persistently while keeping lateral engaged.
    # In pedal mode: software drops long and emits pccDisabled event.
    # In non-pedal mode: stock CC handles its own brake disengage.
    brake_rising_edge = real_brake_pressed and not self.preap_brake_pressed_prev
    if use_pedal:
      if brake_rising_edge and self.cruiseEnabled and self.enableLongControl:
        carlog.warning("BRAKE rising edge — dropping longitudinal, keeping lateral")
        self.enableLongControl = False
        self.enableJustCC = True
        self.pending_enable = False
        self.pedal_speed_kph = 0.0
        self.longCtrlEvent = "pccDisabled"
    self.preap_brake_pressed_prev = real_brake_pressed

    return button_events

  def check_can_engage(self, door_open, gear_shifter, seatbelt_unlatched):
    """Check engagement prerequisites. Resets state if blocked.

    Returns True if engagement is allowed.
    """
    can_engage = (not door_open) and (gear_shifter == structs.CarState.GearShifter.drive) and (not seatbelt_unlatched)
    if not can_engage and self.cruiseEnabled:
      carlog.warning("ENGAGE BLOCKED — can_engage=False: doorOpen=%s gear=%s seatbelt=%s | resetting cruiseEnabled",
                     door_open, gear_shifter, seatbelt_unlatched)
      self.cruiseEnabled = False
      self.enableLongControl = False
      self.enableJustCC = False
      self.pending_enable = False
    return can_engage

  # --- Internal helpers ---

  def _handle_double_pull(self, curr_time_ms, v_ego, speed_units,
                          use_pedal, pedal_long_allowed, long_control_allowed):
    """Handle MAIN press with double-pull detection enabled."""
    # Update timing FIRST, then check (order matches Tinkla)
    self.prev_stalk_pull_time_ms = self.stalk_pull_time_ms
    self.stalk_pull_time_ms = curr_time_ms
    double_pull = (
      self.stalk_pull_time_ms - self.prev_stalk_pull_time_ms
      < self.double_pull_window_ms
    )

    if double_pull:
      carlog.warning("STALK double-pull detected (dt=%dms, window=%dms)",
                     self.stalk_pull_time_ms - self.prev_stalk_pull_time_ms,
                     self.double_pull_window_ms)
      # Double pull detected — enable lateral + longitudinal.
      # long_control_allowed is True for both pedal and non-pedal modes
      # (Tinkla LONG_module pattern: PCC or ACC, mutually exclusive).
      self.cruiseEnabled = True
      self.pending_enable = False
      self.enableLongControl = long_control_allowed
      self.enableJustCC = not long_control_allowed
      if pedal_long_allowed:
        self.longCtrlEvent = "pccEnabled"
        self.pedal_speed_kph = self._capture_target_speed(v_ego, speed_units)
      else:
        self.pedal_speed_kph = 0.0
        if not use_pedal:
          self.preap_cc_engage_needed = True
          self.preap_last_cc_spoof_ms = curr_time_ms
    else:
      carlog.warning("STALK first pull — lateral only, waiting for double (window=%dms)",
                     self.double_pull_window_ms)
      # First pull - engage lateral immediately, wait for possible double
      was_long_active = self.enableLongControl
      self.cruiseEnabled = True
      self.enableLongControl = False
      self.enableJustCC = True
      self.pedal_speed_kph = 0.0
      self.pending_enable = True
      if was_long_active:
        self.longCtrlEvent = "pccDisabled"
      if not use_pedal:
        self.preap_cc_cancel_needed = True
        self.preap_last_cc_spoof_ms = curr_time_ms

  def _make_button_event(self, cruise_buttons, prev_cruise_buttons, curr_time_ms,
                         v_ego, speed_units, use_pedal):
    """Create a ButtonEvent for a button state change."""
    carlog.warning("STALK button change: %d -> %d", prev_cruise_buttons, cruise_buttons)
    be = structs.CarState.ButtonEvent()
    be.pressed = cruise_buttons != CruiseButtons.IDLE

    # Determine which button for event type
    state = cruise_buttons if be.pressed else prev_cruise_buttons

    if state == CruiseButtons.MAIN:
      be.type = ButtonType.setCruise
      if be.pressed:
        self.last_stalk_non_cancel_ms = curr_time_ms

    elif state == CruiseButtons.CANCEL:
      # Push away - cancel everything.
      # Ignore a short synthetic cancel pulse generated by pedal-over-CC
      # immediately after MAIN/RES/DECEL press edges.
      # This prevents self-cancel when spoofing stock-CC cancellation.
      is_possible_auto_cancel = (
        (self.enableLongControl and (curr_time_ms - self.last_stalk_non_cancel_ms) < 600)
        or ((curr_time_ms - self.preap_last_cc_spoof_ms) < 300)
      )
      if not is_possible_auto_cancel:
        carlog.warning("STALK CANCEL — disabling all control")
        be.type = ButtonType.cancel
        was_long_active = self.enableLongControl
        self.cruiseEnabled = False
        self.enableLongControl = False
        self.enableJustCC = False
        self.pending_enable = False
        self.pedal_speed_kph = 0.0
        # Reset timing to prevent false double-pulls after cancel
        self.stalk_pull_time_ms = 0
        self.prev_stalk_pull_time_ms = -1000
        if was_long_active:
          self.longCtrlEvent = "pccDisabled"
      else:
        be.type = ButtonType.unknown

    elif CruiseButtons.is_accel(state):
      # Up - accelerate (Tinkla PCC_module.py lines 194-207)
      be.type = ButtonType.accelCruise
      if be.pressed:
        self.last_stalk_non_cancel_ms = curr_time_ms
        if not use_pedal and self.cruiseEnabled and not self.enableLongControl:
          self.enableLongControl = True
          self.pending_enable = False
        # Only adjust speed on press edge (not release) to avoid double-increment
        if self.enableLongControl:
          speed_uom_kph = CV.MPH_TO_KPH if speed_units == "MPH" else 1.0
          actual_kph = int(v_ego * CV.MS_TO_KPH / speed_uom_kph + 0.5) * speed_uom_kph
          if state == CruiseButtons.RES_ACCEL:
            self.pedal_speed_kph = max(self.pedal_speed_kph, actual_kph) + speed_uom_kph
          else:  # RES_ACCEL_2ND
            self.pedal_speed_kph = max(self.pedal_speed_kph, actual_kph) + 5 * speed_uom_kph
          self.pedal_speed_kph = min(self.pedal_speed_kph, 270.0)

    elif CruiseButtons.is_decel(state):
      # Down - decelerate (Tinkla PCC_module.py lines 204-207)
      be.type = ButtonType.decelCruise
      if be.pressed:
        self.last_stalk_non_cancel_ms = curr_time_ms
        if not use_pedal and self.cruiseEnabled and not self.enableLongControl:
          self.enableLongControl = True
          self.pending_enable = False
        # Only adjust speed on press edge (not release) to avoid double-decrement
        if self.enableLongControl:
          speed_uom_kph = CV.MPH_TO_KPH if speed_units == "MPH" else 1.0
          if state == CruiseButtons.DECEL_SET:
            self.pedal_speed_kph = self.pedal_speed_kph - speed_uom_kph
          else:  # DECEL_2ND
            self.pedal_speed_kph = self.pedal_speed_kph - 5 * speed_uom_kph
          self.pedal_speed_kph = max(self.pedal_speed_kph, 0.0)

    else:
      be.type = ButtonType.unknown

    return be

  @staticmethod
  def _capture_target_speed(v_ego, speed_units):
    """Capture current speed as target speed (Tinkla PCC_module.py line 172-178)."""
    speed_uom_kph = CV.MPH_TO_KPH if speed_units == "MPH" else 1.0
    current_speed_kph = int(v_ego * CV.MS_TO_KPH / speed_uom_kph + 0.5) * speed_uom_kph
    return max(current_speed_kph, 0.0)
