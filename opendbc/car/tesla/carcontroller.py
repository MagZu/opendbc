import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.carlog import carlog
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven, TeslaCANPreAP
from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS, CAR
from opendbc.car.vehicle_model import VehicleModel
from numpy import interp, clip

# Import Tinkla config and pedal constants
try:
  from opendbc.car.tesla.tinkla_conf import (
    tinkla_conf,
    PEDAL_DI_MIN, PEDAL_DI_ZERO, PEDAL_DI_PRESSED,
    PEDAL_BP, PEDAL_V_DEFAULT,
    ACCEL_MAX,
  )
  TINKLA_AVAILABLE = True
except ImportError:
  TINKLA_AVAILABLE = False
  tinkla_conf = None
  PEDAL_DI_PRESSED = 2  # Fallback default

# Import CruiseButtons for cruise spam fallback
try:
  from opendbc.car.tesla.values import CruiseButtons
  CRUISE_BUTTONS_AVAILABLE = True
except ImportError:
  CRUISE_BUTTONS_AVAILABLE = False

# Pedal rate limiter: max DI change per 20ms step (pedal sends at 50 Hz).
# Prevents WOT-on-engage: even with kf=1.0 feedforward, the physical pedal
# ramps over ~0.3-0.6s instead of jumping instantly.
# 2.5 DI/step = 125 DI/s.  P85+ at highway (max=75 DI): 0→full in 0.6s.
PEDAL_RAMP_RATE = 2.5

# Fallback pedal constants (used when tinkla_conf unavailable)
# From Tinkla tunes.py
PEDAL_DI_MIN_DEFAULT = -5
PEDAL_DI_ZERO_DEFAULT = 0
PEDAL_CALIB_FACTOR_DEFAULT = 1.0
PEDAL_CALIB_ZERO_DEFAULT = 0.0
PEDAL_ZERO_DEFAULT = PEDAL_CALIB_ZERO_DEFAULT - 1.0 / PEDAL_CALIB_FACTOR_DEFAULT  # = -1.0

def _transform_di_to_pedal(val):
  """Default DI→pedal transform when tinkla_conf unavailable. Matches Tinkla tunes.py."""
  return PEDAL_ZERO_DEFAULT + (val - PEDAL_DI_ZERO_DEFAULT) / PEDAL_CALIB_FACTOR_DEFAULT


def get_safety_CP():
  # We use the TESLA_MODEL_Y platform for lateral limiting to match safety
  # A Model 3 at 40 m/s using the Model Y limits sees a <0.3% difference in max angle (from curvature factor)
  from opendbc.car.tesla.interface import CarInterface
  return CarInterface.get_non_essential_params("TESLA_MODEL_Y")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCAN(self.packer)

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())
    
    # ============================================
    # Pedal Control State (Tinkla PCC_module port)
    # ============================================
    self.prev_pedal_di = 0.0      # Previous pedal value in DI units
    self.prev_v_ego = 0.0         # Previous vehicle speed
    

    # State tracking
    self.prev_enable_long_control = False
    self.prev_requested_long = False
    self.preap_cancel_pending = False
    self.preap_engage_pending = False
    self.prev_preap_long_active = False
    self.preap_long_engage_frame = -1000000

    if CP.carFingerprint in LEGACY_CARS:
      if CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, CAR.TESLA_MODEL_S_PREAP):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.packers = {CANBUS.party: CANPacker(dbc_names[Bus.party]), CANBUS.powertrain: CANPacker(dbc_names[Bus.pt])}
      
      if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
        self.packers[CANBUS.autopilot_party] = CANPacker(dbc_names[Bus.party])
        self.pedal_packer = CANPacker("comma_pedal")
        self.tesla_can = TeslaCANPreAP(self.packers, self.pedal_packer)
        
        # Configure pedal CAN bus from tinkla_conf
        if TINKLA_AVAILABLE and tinkla_conf:
          self.tesla_can.pedal_can_bus = tinkla_conf.pedal_can_bus
        else:
          self.tesla_can.pedal_can_bus = 2  # Default to bus 2
      else:
        self.tesla_can = TeslaCANRaven(self.packers)
        
      from opendbc.car.tesla.interface import CarInterface
      self.VM = VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3"))

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # Tesla EPS enforces disabling steering on heavy lateral override force.
    # When enabling in a tight curve, we wait until user reduces steering force to start steering.
    # Canceling is done on rising edge and is handled generically with CC.cruiseControl.cancel
    lat_active = CC.latActive and CS.hands_on_level < 3

    if self.frame % 2 == 0:
      # Angular rate limit based on speed
      self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                          lat_active, CarControllerParams, self.VM)
      if self.CP.carFingerprint in LEGACY_CARS:
          cntr = (self.frame // 2) % 16
          can_sends.append(self.tesla_can.create_steering_control(cntr, self.apply_angle_last, lat_active))
      else:
        can_sends.append(self.tesla_can.create_steering_control(self.apply_angle_last, lat_active))

    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
        if self.frame % 2 == 0:
            cntr = (self.frame // 2) % 16
            can_sends.append(self.tesla_can.create_epas_control(cntr, 1)) # Mode 1 = WITH_ANGLE
    elif self.frame % 10 == 0 and self.CP.carFingerprint not in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1):
      # Pre-AP might need this if it has EPAS, let's include it for now if not strictly forbidden
      # Tinkla sends this.
      cntr = (self.frame // 10) % 16
      can_sends.append(self.tesla_can.create_steering_allowed(cntr))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
        # ==============================================
        # Pre-AP Longitudinal Control (Tinkla port)
        # Pedal at 50Hz (frame % 2) to match Tinkla LONG_module.py line 213
        # ==============================================
        # Get engagement state (used for both pedal and pedal-over-CC)
        cs_cruise_enabled = getattr(CS, 'cruiseEnabled', False)
        cs_enable_long = getattr(CS, 'enableLongControl', False)
        requested_long = cs_cruise_enabled and cs_enable_long
        long_active = requested_long and CC.longActive
        use_pedal = TINKLA_AVAILABLE and tinkla_conf and tinkla_conf.use_pedal
        pedal_factor = float(tinkla_conf.pedal_factor) if (TINKLA_AVAILABLE and tinkla_conf) else 1.0
        pedal_transform_valid = bool(np.isfinite(pedal_factor) and abs(pedal_factor) > 1e-6)
        pedal_long_allowed = bool(use_pedal and pedal_transform_valid)
        if long_active and not self.prev_preap_long_active:
          self.preap_long_engage_frame = self.frame
          self.prev_pedal_di = 0.0  # Rate limiter starts from zero on fresh engage

        # ==============================================
        # Pedal Over CC: one-shot CANCEL to keep stock CC unlatch
        # in pedal mode. Trigger on:
        #  - requested-long engage edge
        #  - requested-long disengage edge
        #  - real stalk press edges for engage/speed change
        # Do NOT use CC.cruiseControl.cancel directly here, as controlsd
        # keeps it asserted when pcmCruise is False.
        # ==============================================
        if pedal_long_allowed:
          if (not self.prev_requested_long) and requested_long and CS.out.cruiseState.enabled:
            self.preap_cancel_pending = True
          if self.prev_requested_long and (not requested_long) and CS.out.cruiseState.enabled:
            self.preap_cancel_pending = True

          if CRUISE_BUTTONS_AVAILABLE:
            cruise_buttons = getattr(CS, "cruise_buttons", CruiseButtons.IDLE)
            prev_cruise_buttons = getattr(CS, "prev_cruise_buttons", CruiseButtons.IDLE)
            stalk_press_edge = cruise_buttons != prev_cruise_buttons and cruise_buttons != CruiseButtons.IDLE
            if stalk_press_edge:
              pedal_over_cc_button = (
                cruise_buttons == CruiseButtons.MAIN
                or CruiseButtons.is_accel(cruise_buttons)
                or CruiseButtons.is_decel(cruise_buttons)
              )
              if pedal_over_cc_button and requested_long and CS.out.cruiseState.enabled:
                self.preap_cancel_pending = True

        if self.preap_cancel_pending and self.frame % 10 == 0:
          msg_stw = getattr(CS, 'msg_stw_actn_req', None)
          if msg_stw is not None:
            stlk_counter = (int(msg_stw.get('MC_STW_ACTN_RQ', 0)) + 1) % 16
            can_sends.insert(0, self.tesla_can.create_action_request(
              CruiseButtons.CANCEL, CANBUS.party, stlk_counter, msg_stw))
            self.preap_cancel_pending = False
        elif self.preap_engage_pending and self.frame % 10 == 0:
          msg_stw = getattr(CS, 'msg_stw_actn_req', None)
          if msg_stw is not None:
            stlk_counter = (int(msg_stw.get('MC_STW_ACTN_RQ', 0)) + 1) % 16
            can_sends.insert(0, self.tesla_can.create_action_request(
              CruiseButtons.RES_ACCEL, CANBUS.party, stlk_counter, msg_stw))
            self.preap_engage_pending = False

        self.prev_requested_long = requested_long

        # Non-pedal CC commands: consume flags set by carstate stalk handler
        if not pedal_long_allowed:
          if getattr(CS, 'preap_cc_cancel_needed', False):
            self.preap_cancel_pending = True
            CS.preap_cc_cancel_needed = False
          if getattr(CS, 'preap_cc_engage_needed', False):
            self.preap_engage_pending = True
            CS.preap_cc_engage_needed = False

        # Gate ALL pedal sends on pedal availability.
        # When pedal is not responding (unplugged or absent), sending 0x551 to a dead
        # bus fills its TX queue.  can_tx_check_min_slots_free() checks ALL bus queues,
        # so one full queue blocks USB sendcan for ALL buses — including bus 0 steering.
        # Matches tinkla: PCC_module._update_pedal_state() gates on pedal_idx changes.
        pedal_responding = not getattr(CS, 'pedal_timeout', True)

        if self.frame % 2 == 0:
           self.prev_enable_long_control = cs_enable_long

           if long_active and pedal_long_allowed:
             # ============================================
             # Mode 1: Comma Pedal Control
             # Matches Tinkla Pre-AP behavior: always send commands when
             # use_pedal is True and long is active. Tinkla's pcc_available
             # is always True for Pre-AP (autopilot_disabled=True).
             # ============================================
            try:
              if CS.out.gasPressed:
                # Tinkla PCC_module.py line 294: if CS.out.gasPressed, stop commanding
                # This is the SAFE approach - let the human have full control
                can_sends.append(self.tesla_can.create_pedal_command(0, enable=0))
              else:
                accel_request = float(actuators.accel)
                target_speed_kph = float(getattr(CS, "pedal_speed_kph", 0.0))
                pedal_cmd = self._calc_pedal_command(accel_request, CS.out.vEgo, target_speed_kph)
                can_sends.append(self.tesla_can.create_pedal_command(pedal_cmd, enable=1))

                # Max regen warning: alert driver when pedal is at/near max regen
                # (they need to use the brake pedal for more deceleration).
                # Tinkla PCC_module.py line 353: trigger at 95% of PEDAL_DI_MIN, suppress for 2s after engage.
                pedal_di_min = PEDAL_DI_MIN if TINKLA_AVAILABLE else PEDAL_DI_MIN_DEFAULT
                engage_elapsed = (self.frame - self.preap_long_engage_frame) * 0.01  # frames to seconds at 100Hz
                if self.prev_pedal_di <= 0.95 * pedal_di_min and engage_elapsed > 2.0:
                  CS.pccEvent = "pedalMaxRegen"
                else:
                  CS.pccEvent = None
            except Exception:
              # Fail-safe: on any unexpected pedal path exception, send disabled pedal.
              carlog.exception("Pre-AP pedal command path failed; sending disabled pedal command")
              idle_pedal = tinkla_conf.di_to_pedal(PEDAL_DI_ZERO) if tinkla_conf else _transform_di_to_pedal(PEDAL_DI_ZERO_DEFAULT)
              can_sends.append(self.tesla_can.create_pedal_command(idle_pedal, enable=0))
              self.prev_pedal_di = 0.0

           elif use_pedal and not pedal_transform_valid:
             # Safety gate: block pedal actuation when pedal transform is invalid.
             idle_pedal = tinkla_conf.di_to_pedal(PEDAL_DI_ZERO) if tinkla_conf else _transform_di_to_pedal(PEDAL_DI_ZERO_DEFAULT)
             can_sends.append(self.tesla_can.create_pedal_command(idle_pedal, enable=0))
             self.prev_pedal_di = 0.0

           else:
             # ============================================
             # Steering Only (Single Pull) or Not Engaged
             # Send idle pedal keepalive to prevent firmware fault
             # Tinkla PCC_module.py line 132: sends reset at frame % 50 (2Hz)
             # ============================================
             if use_pedal:
               if pedal_responding:
                 # Pedal is alive — send idle keepalive at 50Hz (every frame %2)
                 idle_pedal = tinkla_conf.di_to_pedal(PEDAL_DI_ZERO) if tinkla_conf else _transform_di_to_pedal(PEDAL_DI_ZERO_DEFAULT)
                 can_sends.append(self.tesla_can.create_pedal_command(idle_pedal, enable=0))
               elif self.frame % 100 == 0:
                 # Pedal not responding — send disabled reset at 1Hz to wake it up.
                 # Low rate avoids flooding a dead bus (can_tx_check_min_slots_free
                 # blocks ALL buses if any one queue fills).  Tinkla uses 2Hz here
                 # (frame%50) but 1Hz is safer for dead-bus tolerance.
                 idle_pedal = tinkla_conf.di_to_pedal(PEDAL_DI_ZERO) if tinkla_conf else _transform_di_to_pedal(PEDAL_DI_ZERO_DEFAULT)
                 can_sends.append(self.tesla_can.create_pedal_command(idle_pedal, enable=0))
             # Reset state when not active
             self.prev_pedal_di = 0.0

        self.prev_preap_long_active = long_active

      elif self.frame % 4 == 0:
        # Non-Pre-AP longitudinal control (HW1/HW2/HW3 with DAS_control) at 25Hz
        state = 13 if CC.cruiseControl.cancel else 4  # 4=ACC_ON, 13=ACC_CANCEL_GENERIC_SILENT
        accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        cntr = (self.frame // 4) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive))

    else:
      # Not openpilotLongitudinalControl - handle cancel
      if CC.cruiseControl.cancel:
        if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
           if not getattr(CS, 'pedal_timeout', True):
             idle_pedal = tinkla_conf.di_to_pedal(PEDAL_DI_ZERO) if (TINKLA_AVAILABLE and tinkla_conf) else _transform_di_to_pedal(PEDAL_DI_ZERO_DEFAULT)
             can_sends.append(self.tesla_can.create_pedal_command(idle_pedal, enable=0))
        else:
           cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
           can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False))

    # TODO: HUD control
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends

  # ============================================
  # Pedal Control Logic (Ported from Tinkla PCC_module.py)
  # ============================================

  def _calc_pedal_command(self, accel_request: float, v_ego: float, target_speed_kph: float | None = None) -> float:
    """
    Convert acceleration request (m/s^2) to comma pedal voltage.

    Architecture (FrogPilot/OPGM Bolt-inspired, feedforward-dominant):
      1. Linear map: accel -> DI pedal units via [regen_decel, 0, ACCEL_MAX]
      2. Clamp to trim profile max (P85+/P85/S85/S60 speed-dependent)
      3. Rate limiter: ±PEDAL_RAMP_RATE DI/step (WOT-on-engage defense)
      4. Calibration transform: DI -> pedal voltage

    With kf=1.0, actuators.accel ≈ a_target + slow_integral_trim.
    The MPC plan is jerk-constrained and smooth; the rate limiter catches
    any remaining transients (engage edges, planner mode switches).
    """
    if not TINKLA_AVAILABLE or not tinkla_conf:
      # Fallback: simple linear mapping if tinkla_conf unavailable
      pedal_di = float(clip(interp(accel_request, [-1.5, 0., 2.0], [-5., 0., 100.]), -5, 100))
      pedal_di = float(clip(pedal_di,
                            self.prev_pedal_di - PEDAL_RAMP_RATE,
                            self.prev_pedal_di + PEDAL_RAMP_RATE))
      self.prev_pedal_di = pedal_di
      return _transform_di_to_pedal(pedal_di)

    # Trim-specific max pedal (P85+, P85, S85, S60, Generic)
    pedal_profile = tinkla_conf.get_pedal_profile_values()
    max_pedal_value = float(interp(v_ego, PEDAL_BP, pedal_profile))

    # Speed-dependent regen limit (more regen available at higher speeds)
    regen_decel = float(interp(v_ego, [5., 15.], [-1.2, -1.45]))

    # Linear mapping: accel (m/s^2) -> DI pedal units
    # With kf=1.0 feedforward, accel_request ≈ a_target + integral_trim,
    # so this mapping smoothly covers the full regen-to-accel range.
    accel_bp = [regen_decel, 0.0, ACCEL_MAX]
    accel_v = [PEDAL_DI_MIN, 0.0, max_pedal_value]
    pedal_di = float(interp(accel_request, accel_bp, accel_v))

    # Clamp to trim profile limits
    pedal_di = float(clip(pedal_di, PEDAL_DI_MIN, max_pedal_value))

    # Rate limiter: cap DI change per step to prevent sudden jumps.
    # Primary WOT-on-engage defense — even with kf=1.0, pedal ramps smoothly.
    pedal_di = float(clip(pedal_di,
                          self.prev_pedal_di - PEDAL_RAMP_RATE,
                          self.prev_pedal_di + PEDAL_RAMP_RATE))

    # Transform DI -> pedal voltage via calibration
    pedal_cmd = tinkla_conf.di_to_pedal(pedal_di)

    self.prev_pedal_di = pedal_di
    self.prev_v_ego = v_ego

    return pedal_cmd


