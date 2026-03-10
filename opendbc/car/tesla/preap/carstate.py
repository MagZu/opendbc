"""
Pre-AP CarState update and CAN parser config for Pre-AP Model S.

Extracted from carstate.py so upstream changes to the AP1+ legacy
update path and parser config don't conflict with Pre-AP logic.
"""
import copy
import math
import time

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import DBC, CANBUS, GEAR_MAP, STEER_THRESHOLD
from opendbc.car.tesla.nap_params import NAPParamKeys
from opendbc.car.tesla.nap_conf import nap_conf, PEDAL_DI_PRESSED

try:
  from openpilot.common.params import Params as _NAPParams
  _nap_params = _NAPParams()
except ImportError:
  _nap_params = None


def _current_time_millis():
  return int(round(time.time() * 1000))


def update_preap(cs, can_parsers):
  """Update CarState for Pre-AP Model S.

  Args:
    cs: CarState instance (for engagement FSM, pedal feedback, bridge attrs)
    can_parsers: dict of CAN parsers keyed by Bus enum

  Returns:
    structs.CarState with all fields populated
  """
  cp_ap_party = can_parsers[Bus.ap_party]
  cp_pt = can_parsers[Bus.pt]
  cp_chassis = can_parsers[Bus.chassis]
  ret = structs.CarState()

  # Vehicle speed
  ret.vEgoRaw = cp_chassis.vl["ESP_B"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
  ret.vEgo, ret.aEgo = cs.update_speed_kf(ret.vEgoRaw)

  # Gas pedal — Pre-AP uses a small DI threshold to avoid sticky overrides
  # (Tinkla: strict >0 on DI_pedalPos keeps gas override active)
  ret.gasPressed = cp_pt.vl["DI_torque1"]["DI_pedalPos"] > PEDAL_DI_PRESSED

  # Brake pedal
  ret.brake = 0
  real_brake_pressed = cp_chassis.vl["BrakeMessage"]["driverBrakeStatus"] == 2
  ret.brakePressed = real_brake_pressed

  # Steering wheel — Pre-AP uses chassis EPAS
  epas_status = cp_chassis.vl["EPAS_sysStatus"]
  cs.hands_on_level = epas_status["EPAS_handsOnLevel"]
  ret.steeringAngleDeg = -epas_status["EPAS_internalSAS"]
  ret.steeringRateDeg = -cp_chassis.vl["STW_ANGLHP_STAT"]["StW_AnglHP_Spd"]
  ret.steeringTorque = -epas_status["EPAS_torsionBarTorque"]
  ret.steeringPressed = cs.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)

  eac_status = cs.can_defines["EPAS_sysStatus"]["EPAS_eacStatus"].get(int(epas_status["EPAS_eacStatus"]), None)
  ret.steerFaultPermanent = eac_status == "EAC_FAULT"
  # On Pre-AP, EAC_INHIBITED is the normal EPS idle state (no AP ECU present),
  # not a real fault.  Mapping it to steerFaultTemporary creates a deadlock:
  # latActive stays False, so DAS_steeringControlType=0 is sent, so the EPS
  # never transitions to AVAILABLE/ACTIVE.
  ret.steerFaultTemporary = False

  eac_error_code = cs.can_defines["EPAS_sysStatus"]["EPAS_eacErrorCode"].get(int(epas_status["EPAS_eacErrorCode"]), None)
  ret.steeringDisengage = cs.hands_on_level >= 3 or (eac_status == "EAC_INHIBITED" and
                                                      eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")
  cs.engagement.handle_steering_disengage(ret.steeringDisengage)

  # Cruise state
  cruise_state = cs.can_defines["DI_state"]["DI_cruiseState"].get(int(cp_chassis.vl["DI_state"]["DI_cruiseState"]), None)
  speed_units = cs.can_defines["DI_state"]["DI_speedUnits"].get(int(cp_chassis.vl["DI_state"]["DI_speedUnits"]), None)

  ret.cruiseState.available = True  # Always available on Pre-AP

  # Save speed units for stalk button handling
  if speed_units is not None:
    cs.speed_units = speed_units

  if cs.enableLongControl and nap_conf.use_pedal:
    # Pedal mode active: use software-managed target speed (Tinkla PCC_module)
    ret.cruiseState.speed = cs.pedal_speed_kph * CV.KPH_TO_MS
  else:
    # Lateral-only, stock cruise fallback, or not engaged: read dashboard speed.
    # When stock CC is active DI_digitalSpeed shows the set speed;
    # when CC is off it shows current speed (harmless placeholder).
    if speed_units == "KPH":
      ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS, 1e-3)
    elif speed_units == "MPH":
      ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS, 1e-3)

  ret.cruiseState.standstill = False
  ret.standstill = cruise_state == "STANDSTILL"
  ret.accFaulted = cruise_state == "FAULT"

  # Gear
  ret.gearShifter = GEAR_MAP[cs.can_defines["DI_torque2"]["DI_gear"].get(int(cp_chassis.vl["DI_torque2"]["DI_gear"]), "DI_GEAR_INVALID")]

  # Doors
  DOORS = ["DOOR_STATE_FL", "DOOR_STATE_FR", "DOOR_STATE_RL", "DOOR_STATE_RR", "DOOR_STATE_FrontTrunk", "BOOT_STATE"]
  ret.doorOpen = any((cs.can_defines["GTW_carState"][door].get(int(cp_chassis.vl["GTW_carState"][door]), "OPEN") == "OPEN") for door in DOORS)

  # Blinkers
  ret.leftBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorLStatus"] == 1
  ret.rightBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorRStatus"] == 1

  # Seatbelt — SDM1 (0x201) collides with Comma Pedal, hardcode for now
  # TODO: Implement safe check using message size or bus if possible.
  ret.seatbeltUnlatched = False

  # AEB/LKAS — Pre-AP has no DAS ECU
  ret.stockAeb = False
  ret.stockLkas = False

  # ============================================
  # Buttons + Engagement FSM
  # ============================================
  cs.prev_cruise_buttons = cs.cruise_buttons
  cs.cruise_buttons = int(cp_chassis.vl["STW_ACTN_RQ"]["SpdCtrlLvr_Stat"])
  # Save full STW_ACTN_RQ message for spoofing cancel commands (Tinkla carstate.py line 432)
  cs.msg_stw_actn_req = copy.copy(cp_chassis.vl["STW_ACTN_RQ"])

  # Read follow distance dial from cruise stalk
  if _nap_params is not None:
    dtr_dist = int(cp_chassis.vl["STW_ACTN_RQ"]["DTR_Dist_Rq"])
    if dtr_dist != 255:  # 255 = SNA (no stalk input)
      stalk_follow = min((dtr_dist // 33) + 1, 7)
      if stalk_follow != cs.prev_stalk_follow:
        _nap_params.put(NAPParamKeys.FOLLOW_DISTANCE, stalk_follow)
        cs.prev_stalk_follow = stalk_follow

  curr_time_ms = _current_time_millis()
  use_pedal = nap_conf.use_pedal
  pedal_factor = float(nap_conf.pedal_factor)
  pedal_transform_valid = math.isfinite(pedal_factor) and abs(pedal_factor) > 1e-6
  pedal_long_allowed = use_pedal and pedal_transform_valid
  long_control_allowed = (not use_pedal) or pedal_transform_valid

  # Delegate engagement logic to PreAPEngagement module
  button_events = cs.engagement.process_buttons(
    cs.cruise_buttons, cs.prev_cruise_buttons, curr_time_ms,
    ret.vEgo, cs.speed_units, use_pedal, pedal_long_allowed,
    long_control_allowed, real_brake_pressed)
  # Suppress brakePressed so openpilot's generic brake-disengage path doesn't kill lateral
  ret.brakePressed = False
  ret.buttonEvents = button_events

  # Check engagement prerequisites (door, gear, seatbelt)
  can_engage = cs.engagement.check_can_engage(ret.doorOpen, ret.gearShifter, ret.seatbeltUnlatched)
  ret.cruiseState.enabled = cs.engagement.cruiseEnabled and can_engage

  # Bridge engagement state for carcontroller reads
  cs.cruiseEnabled = cs.engagement.cruiseEnabled
  cs.enableLongControl = cs.engagement.enableLongControl
  cs.enableJustCC = cs.engagement.enableJustCC
  cs.pedal_speed_kph = cs.engagement.pedal_speed_kph
  cs.longCtrlEvent = cs.engagement.longCtrlEvent
  cs.preap_cc_cancel_needed = cs.engagement.preap_cc_cancel_needed
  cs.preap_cc_engage_needed = cs.engagement.preap_cc_engage_needed

  # ============================================
  # Comma Pedal Parsing
  # ============================================
  gas_sensor = cp_ap_party.vl.get("GAS_SENSOR", {})
  cs.pedal.update(gas_sensor, curr_time_ms)
  cs.pedal.update_torque(cp_pt.vl.get("DI_torque1", {}))

  # Bridge pedal state for carcontroller reads
  cs.pedal_interceptor_value = cs.pedal.interceptor_value
  cs.pedal_timeout = cs.pedal.timeout

  # In pedal mode, use interceptor threshold for gas override semantics.
  # Matches Tinkla behavior: avoids sticky DI_pedalPos > 0 overrides.
  if nap_conf.use_pedal:
    ret.gasPressed = cs.pedal.gas_pressed

  cs.das_control = None
  cs.cruise_enabled_prev = ret.cruiseState.enabled

  # Propagate max regen flag from carcontroller (set in previous frame)
  ret.pedalMaxRegen = cs.pccEvent == "pedalMaxRegen"

  # Expose pedal-specific long control status for selfdrived alerts.
  # True only when pedal hardware is the longitudinal source (not stock cruise).
  ret.pedalLongActive = cs.enableLongControl and nap_conf.use_pedal

  return ret


def get_preap_can_parsers(CP):
  """CAN parser configuration for Pre-AP Model S.

  Defines which CAN messages to subscribe to and on which buses.
  Pre-AP has no DAS ECU, so AP parsers are empty or pointed at Bus 0.
  Comma Pedal feedback (GAS_SENSOR) uses Bus 2 by default.
  """
  # Bus 0 chassis signals
  chassis_messages = [
    ("ESP_B", 0),
    ("BrakeMessage", 0),
    ("DI_state", 0),
    ("DI_torque2", 0),
    ("GTW_carState", 0),
    ("STW_ANGLHP_STAT", 0),
    ("EPAS_sysStatus", 0),
    ("STW_ACTN_RQ", 0),
    # SDM1 (0x201) excluded — collides with Comma Pedal on Bus 0
    # RCM_status excluded — not present on Pre-AP, causes timeout
  ]

  # Bus 0 powertrain signals
  pt_messages = [
    ("DI_torque1", 0),
    ("ESP_B", 0),
  ]

  # Bus 0 party signals (minimal — keeps parser valid)
  party_messages = [
    ("ESP_B", 0),
  ]

  # Comma Pedal feedback on Bus 2 (or Bus 0 if pedal_can_zero)
  pedal_can_zero = nap_conf.pedal_can_zero
  pedal_bus = 0 if pedal_can_zero else 2
  pedal_messages = [
    # Optional: don't invalidate CAN health if pedal is absent
    ("GAS_SENSOR", math.nan),
  ]

  # Pre-AP has no DAS ECU — AP parsers point at Bus 0 with ESP_B only
  ap_messages = [
    ("ESP_B", 0),
  ]

  return {
    Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], party_messages, CANBUS.party),
    Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], pedal_messages, pedal_bus),
    Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CANBUS.party),
    Bus.ap_pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CANBUS.party),
    Bus.chassis: CANParser(DBC[CP.carFingerprint][Bus.chassis], chassis_messages, CANBUS.party),
  }
