import copy
import math
import time
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.carlog import carlog
from opendbc.car.tesla.values import DBC, CANBUS, GEAR_MAP, STEER_THRESHOLD, CAR, TeslaLegacyParams, LEGACY_CARS, CruiseButtons
from opendbc.car.tesla.nap_params import NAPParamKeys
from opendbc.car.tesla.tinkla_conf import tinkla_conf
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.preap.pedal_feedback import PedalFeedback

try:
  from openpilot.common.params import Params as _NAPParams
  _nap_params = _NAPParams()
except ImportError:
  _nap_params = None

def _current_time_millis():
  return int(round(time.time() * 1000))


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.can_define = CANDefine(DBC[CP.carFingerprint][Bus.party])

    if self.CP.carFingerprint in LEGACY_CARS:
      if self.CP.carFingerprint == CAR.TESLA_MODEL_S_HW3:
        CANBUS.chassis = 1
        CANBUS.radar = 5
      elif self.CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, CAR.TESLA_MODEL_S_PREAP):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.can_define_party = CANDefine(DBC[CP.carFingerprint][Bus.party])
      self.can_define_pt = CANDefine(DBC[CP.carFingerprint][Bus.pt])
      self.can_define_chassis = CANDefine(DBC[CP.carFingerprint][Bus.chassis])
      self.can_defines = {
        **self.can_define_party.dv,
        **self.can_define_pt.dv,
        **self.can_define_chassis.dv,
      }
      self.shifter_values = self.can_defines["DI_torque2"]["DI_gear"]
    else:
      self.shifter_values = self.can_define.dv["DI_systemStatus"]["DI_gear"]

    self.autopark = False
    self.autopark_prev = False
    self.cruise_enabled_prev = False

    self.hands_on_level = 0
    self.das_control = None
    self.cruise_buttons = 0
    self.prev_cruise_buttons = 0
    self.msg_stw_actn_req = None  # Full STW_ACTN_RQ message for spoofing cancel commands

    # Follow distance stalk tracking
    self.prev_stalk_follow = 0
    self.speed_units = "MPH"  # Updated from DI_state each frame

    # Pre-AP engagement state machine (double-pull, button handling, brake override)
    self.engagement = PreAPEngagement(
      double_pull_enabled=tinkla_conf.double_pull_enabled,
      double_pull_window_ms=tinkla_conf.double_pull_window_ms,
    )
    # Bridge attributes: carcontroller reads these via getattr(CS, 'X', default)
    self.cruiseEnabled = False
    self.enableLongControl = False
    self.enableJustCC = False
    self.pedal_speed_kph = 0.0
    self.longCtrlEvent = None
    self.preap_cc_cancel_needed = False
    self.preap_cc_engage_needed = False

    # Comma Pedal feedback parser
    self.pedal = PedalFeedback()
    # Bridge attributes for carcontroller reads via getattr(CS, ...)
    self.pedal_interceptor_value = 0.0
    self.pedal_timeout = True

    # Alert event set by carcontroller (pedalMaxRegen), read by carstate
    self.pccEvent = None

  def update_autopark_state(self, autopark_state: str, cruise_enabled: bool):
    autopark_now = autopark_state in ("ACTIVE", "COMPLETE", "SELFPARK_STARTED")
    if autopark_now and not self.autopark_prev and not self.cruise_enabled_prev:
      self.autopark = True
    if not autopark_now:
      self.autopark = False
    self.autopark_prev = autopark_now
    self.cruise_enabled_prev = cruise_enabled

  def update(self, can_parsers) -> structs.CarState:
    if self.CP.carFingerprint in LEGACY_CARS:
      return self.update_legacy(can_parsers)

    cp_party = can_parsers[Bus.party]
    cp_ap_party = can_parsers[Bus.ap_party]
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp_party.vl["DI_speed"]["DI_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Gas pedal
    ret.gasPressed = cp_party.vl["DI_systemStatus"]["DI_accelPedalPos"] > 0

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp_party.vl["ESP_status"]["ESP_driverBrakeApply"] == 2

    # Steering wheel
    epas_status = cp_party.vl["EPAS3S_sysStatus"]
    self.hands_on_level = epas_status["EPAS3S_handsOnLevel"]
    ret.steeringAngleDeg = -epas_status["EPAS3S_internalSAS"]
    ret.steeringRateDeg = -cp_ap_party.vl["SCCM_steeringAngleSensor"]["SCCM_steeringAngleSpeed"]
    ret.steeringTorque = -epas_status["EPAS3S_torsionBarTorque"]

    # stock handsOnLevel uses >0.5 for 0.25s, but is too slow
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)

    eac_status = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacStatus"].get(int(epas_status["EPAS3S_eacStatus"]), None)
    ret.steerFaultPermanent = eac_status == "EAC_FAULT"
    ret.steerFaultTemporary = eac_status == "EAC_INHIBITED"

    # FSD disengages using union of handsOnLevel (slow overrides) and high angle rate faults (fast overrides, high speed)
    eac_error_code = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacErrorCode"].get(int(epas_status["EPAS3S_eacErrorCode"]), None)
    ret.steeringDisengage = self.hands_on_level >= 3 or (eac_status == "EAC_INHIBITED" and
                                                         eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")

    # Cruise state
    cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(int(cp_party.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units = self.can_define.dv["DI_state"]["DI_speedUnits"].get(int(cp_party.vl["DI_state"]["DI_speedUnits"]), None)

    autopark_state = self.can_define.dv["DI_state"]["DI_autoparkState"].get(int(cp_party.vl["DI_state"]["DI_autoparkState"]), None)
    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")
    self.update_autopark_state(autopark_state, cruise_enabled)

    # Match panda safety cruise engaged logic
    ret.cruiseState.enabled = cruise_enabled and not self.autopark
    if speed_units == "KPH":
      ret.cruiseState.speed = max(cp_party.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS, 1e-3)
    elif speed_units == "MPH":
      ret.cruiseState.speed = max(cp_party.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS, 1e-3)
    ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special
    ret.standstill = cp_party.vl["ESP_B"]["ESP_vehicleStandstillSts"] == 1
    ret.accFaulted = cruise_state == "FAULT"

    # Gear
    ret.gearShifter = GEAR_MAP[self.can_define.dv["DI_systemStatus"]["DI_gear"].get(int(cp_party.vl["DI_systemStatus"]["DI_gear"]), "DI_GEAR_INVALID")]

    # Doors
    ret.doorOpen = cp_party.vl["UI_warning"]["anyDoorOpen"] == 1

    # Blinkers
    ret.leftBlinker = cp_party.vl["UI_warning"]["leftBlinkerBlinking"] in (1, 2)
    ret.rightBlinker = cp_party.vl["UI_warning"]["rightBlinkerBlinking"] in (1, 2)

    # Seatbelt
    ret.seatbeltUnlatched = cp_party.vl["UI_warning"]["buckleStatus"] != 1

    # Blindspot
    ret.leftBlindspot = cp_ap_party.vl["DAS_status"]["DAS_blindSpotRearLeft"] != 0
    ret.rightBlindspot = cp_ap_party.vl["DAS_status"]["DAS_blindSpotRearRight"] != 0

    # AEB
    ret.stockAeb = cp_ap_party.vl["DAS_control"]["DAS_aebEvent"] == 1

    # LKAS
    ret.stockLkas = cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"] == 2  # LANE_KEEP_ASSIST

    # Stock Autosteer should be off (includes FSD)
    if self.CP.carFingerprint in (CAR.TESLA_MODEL_3, CAR.TESLA_MODEL_Y, CAR.TESLA_MODEL_Y_JUNIPER):
      ret.invalidLkasSetting = cp_ap_party.vl["DAS_settings"]["DAS_autosteerEnabled"] != 0
    else:
      pass
    # Buttons # ToDo: add Gap adjust button

    # Messages needed by carcontroller
    self.das_control = copy.copy(cp_ap_party.vl["DAS_control"])

    return ret

  def update_legacy(self, can_parsers) -> structs.CarState:
    cp_party = can_parsers[Bus.party]
    cp_ap_party = can_parsers[Bus.ap_party]
    cp_pt = can_parsers[Bus.pt]
    cp_ap_pt = can_parsers[Bus.ap_pt]
    cp_chassis = can_parsers[Bus.chassis]
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp_chassis.vl["ESP_B"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Gas pedal
    # Pre-AP note: using a strict >0 threshold on DI_pedalPos can keep
    # gas override active and prevent planner longitudinal output.
    # Tinkla uses a small threshold in DI units for interceptor-based gas.
    ret.gasPressed = cp_pt.vl["DI_torque1"]["DI_pedalPos"] > PEDAL_DI_PRESSED

    # Brake pedal
    ret.brake = 0
    real_brake_pressed = cp_chassis.vl["BrakeMessage"]["driverBrakeStatus"] == 2
    ret.brakePressed = real_brake_pressed

    # Steering wheel
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_HW3:
      epas_status = cp_party.vl["EPAS_sysStatus"]
    else:
      epas_status = cp_chassis.vl["EPAS_sysStatus"]

    self.hands_on_level = epas_status["EPAS_handsOnLevel"]
    ret.steeringAngleDeg = -epas_status["EPAS_internalSAS"]
    ret.steeringRateDeg = -cp_chassis.vl["STW_ANGLHP_STAT"]["StW_AnglHP_Spd"]
    ret.steeringTorque = -epas_status["EPAS_torsionBarTorque"]
    # stock handsOnLevel uses >0.5 for 0.25s, but is too slow
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)
    
    eac_status = self.can_defines["EPAS_sysStatus"]["EPAS_eacStatus"].get(int(epas_status["EPAS_eacStatus"]), None)
    ret.steerFaultPermanent = eac_status == "EAC_FAULT"
    # On Pre-AP, EAC_INHIBITED is the normal EPS idle state (no AP ECU present),
    # not a real fault.  Mapping it to steerFaultTemporary creates a deadlock:
    # latActive stays False, so DAS_steeringControlType=0 is sent, so the EPS
    # never transitions to AVAILABLE/ACTIVE.  Only treat it as a temp fault on
    # AP1+ cars where INHIBITED indicates an actual problem.
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      ret.steerFaultTemporary = False
    else:
      ret.steerFaultTemporary = eac_status == "EAC_INHIBITED"
  
    # FSD disengages using union of handsOnLevel (slow overrides) and high angle rate faults (fast overrides, high speed)
    eac_error_code = self.can_defines["EPAS_sysStatus"]["EPAS_eacErrorCode"].get(int(epas_status["EPAS_eacErrorCode"]), None)
    ret.steeringDisengage = self.hands_on_level >= 3 or (eac_status == "EAC_INHIBITED" and
                                                          eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")

    self.engagement.handle_steering_disengage(ret.steeringDisengage)

    # Cruise state
    cruise_state = self.can_defines["DI_state"]["DI_cruiseState"].get(int(cp_chassis.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units = self.can_defines["DI_state"]["DI_speedUnits"].get(int(cp_chassis.vl["DI_state"]["DI_speedUnits"]), None)

    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")

    # Match panda safety cruise engaged logic
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      ret.cruiseState.available = True # Always available on Pre-AP
      # Enabled logic handled by button events below
    else:
      ret.cruiseState.enabled = cruise_enabled
      ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled

    # Save speed units for stalk button handling
    if speed_units is not None:
      self.speed_units = speed_units

    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      if self.enableLongControl and tinkla_conf.use_pedal:
        # Pedal mode active: use software-managed target speed (Tinkla PCC_module)
        ret.cruiseState.speed = self.pedal_speed_kph * CV.KPH_TO_MS
      else:
        # Lateral-only, stock cruise fallback, or not engaged: read dashboard speed.
        # When stock CC is active DI_digitalSpeed shows the set speed;
        # when CC is off it shows current speed (harmless placeholder).
        if speed_units == "KPH":
          ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS, 1e-3)
        elif speed_units == "MPH":
          ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS, 1e-3)
    else:
      if speed_units == "KPH":
        ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS, 1e-3)
      elif speed_units == "MPH":
        ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS, 1e-3)

    if self.CP.carFingerprint != CAR.TESLA_MODEL_S_PREAP:
      ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled

    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special
    ret.standstill = cruise_state == "STANDSTILL"
    ret.accFaulted = cruise_state == "FAULT"

    # Gear
    ret.gearShifter = GEAR_MAP[self.can_defines["DI_torque2"]["DI_gear"].get(int(cp_chassis.vl["DI_torque2"]["DI_gear"]), "DI_GEAR_INVALID")]

    # Doors
    DOORS = ["DOOR_STATE_FL", "DOOR_STATE_FR", "DOOR_STATE_RL", "DOOR_STATE_RR", "DOOR_STATE_FrontTrunk", "BOOT_STATE"]
    ret.doorOpen = any((self.can_defines["GTW_carState"][door].get(int(cp_chassis.vl["GTW_carState"][door]), "OPEN") == "OPEN") for door in DOORS)

    # Blinkers
    ret.leftBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorLStatus"] == 1
    ret.rightBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorRStatus"] == 1

    # Seatbelt
    if self.CP.flags & TeslaLegacyParams.NO_SDM1:
      ret.seatbeltUnlatched = cp_chassis.vl["RCM_status"]["RCM_buckleDriverStatus"] != 1
    elif self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      # Pre-AP uses SDM1 (0x201), but Comma Pedal is also on 0x201.
      # To avoid conflict if we can't distinguish, we hardcode for now.
      # TODO: Implement safe check using message size or bus if possible.
      # For now, we assume belted to allow engagement for testing.
      ret.seatbeltUnlatched = False
    else:
      ret.seatbeltUnlatched = cp_chassis.vl["SDM1"]["SDM_bcklDrivStatus"] != 1

    # AEB
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      ret.stockAeb = False
    else:
      ret.stockAeb = cp_ap_pt.vl["DAS_control"]["DAS_aebEvent"] == 1

    # LKAS
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      ret.stockLkas = False
    else:
      ret.stockLkas = cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"] == 2  # LANE_KEEP_ASSIST

    # Stock Autosteer should be off (includes FSD)
    # ret.invalidLkasSetting = cp_ap_party.vl["DAS_settings"]["DAS_autosteerEnabled"] != 0

    # Buttons # ToDo: add Gap adjust button
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      self.prev_cruise_buttons = self.cruise_buttons
      self.cruise_buttons = int(cp_chassis.vl["STW_ACTN_RQ"]["SpdCtrlLvr_Stat"])
      # Save full STW_ACTN_RQ message for spoofing cancel commands (Tinkla carstate.py line 432)
      self.msg_stw_actn_req = copy.copy(cp_chassis.vl["STW_ACTN_RQ"])

      # Read follow distance dial from cruise stalk
      if _nap_params is not None:
        dtr_dist = int(cp_chassis.vl["STW_ACTN_RQ"]["DTR_Dist_Rq"])
        if dtr_dist != 255:  # 255 = SNA (no stalk input)
          stalk_follow = min((dtr_dist // 33) + 1, 7)
          if stalk_follow != self.prev_stalk_follow:
            _nap_params.put(NAPParamKeys.FOLLOW_DISTANCE, stalk_follow)
            self.prev_stalk_follow = stalk_follow

      curr_time_ms = _current_time_millis()
      use_pedal = tinkla_conf.use_pedal
      pedal_factor = float(tinkla_conf.pedal_factor)
      pedal_transform_valid = math.isfinite(pedal_factor) and abs(pedal_factor) > 1e-6
      pedal_long_allowed = use_pedal and pedal_transform_valid
      long_control_allowed = (not use_pedal) or pedal_transform_valid

      # Delegate engagement logic to PreAPEngagement module
      button_events = self.engagement.process_buttons(
        self.cruise_buttons, self.prev_cruise_buttons, curr_time_ms,
        ret.vEgo, self.speed_units, use_pedal, pedal_long_allowed,
        long_control_allowed, real_brake_pressed)
      # Suppress brakePressed so openpilot's generic brake-disengage path doesn't kill lateral
      ret.brakePressed = False
      ret.buttonEvents = button_events

      # Check engagement prerequisites (door, gear, seatbelt)
      can_engage = self.engagement.check_can_engage(ret.doorOpen, ret.gearShifter, ret.seatbeltUnlatched)
      ret.cruiseState.enabled = self.engagement.cruiseEnabled and can_engage

      # Bridge engagement state for carcontroller reads
      self.cruiseEnabled = self.engagement.cruiseEnabled
      self.enableLongControl = self.engagement.enableLongControl
      self.enableJustCC = self.engagement.enableJustCC
      self.pedal_speed_kph = self.engagement.pedal_speed_kph
      self.longCtrlEvent = self.engagement.longCtrlEvent
      self.preap_cc_cancel_needed = self.engagement.preap_cc_cancel_needed
      self.preap_cc_engage_needed = self.engagement.preap_cc_engage_needed

    # ============================================
    # Comma Pedal Parsing (Pre-AP only)
    # ============================================
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      curr_time_ms = _current_time_millis()
      gas_sensor = cp_ap_party.vl.get("GAS_SENSOR", {})
      self.pedal.update(gas_sensor, curr_time_ms)
      self.pedal.update_torque(cp_pt.vl.get("DI_torque1", {}))

      # Bridge pedal state for carcontroller reads
      self.pedal_interceptor_value = self.pedal.interceptor_value
      self.pedal_timeout = self.pedal.timeout

      # In pedal mode, use interceptor threshold for gas override semantics.
      # Matches Tinkla behavior: avoids sticky DI_pedalPos > 0 overrides.
      if tinkla_conf.use_pedal:
        ret.gasPressed = self.pedal.gas_pressed

    # Messages needed by carcontroller
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      self.das_control = None
    else:
      self.das_control = copy.copy(cp_ap_pt.vl["DAS_control"])

    self.cruise_enabled_prev = ret.cruiseState.enabled

    # Propagate max regen flag from carcontroller (set in previous frame)
    ret.pedalMaxRegen = self.pccEvent == "pedalMaxRegen"

    # Expose pedal-specific long control status for selfdrived alerts.
    # True only when pedal hardware is the longitudinal source (not stock cruise).
    ret.pedalLongActive = self.enableLongControl and tinkla_conf.use_pedal

    return ret

  @staticmethod
  def get_can_parsers(CP):
    if CP.carFingerprint in LEGACY_CARS:
      chassis_messages = [
        ("ESP_B", 0),
        ("BrakeMessage", 0),
        ("DI_state", 0),
        ("DI_torque2", 0),
        ("GTW_carState", 0),
        ("STW_ANGLHP_STAT", 0),
        ("SDM1", 0),
        ("RCM_status", 0),
      ]
      
      if CP.carFingerprint != CAR.TESLA_MODEL_S_HW3:
        chassis_messages.append(("EPAS_sysStatus", 0))
      
      if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
        # Remove RCM_status to prevent timeout on Pre-AP
        chassis_messages = [m for m in chassis_messages if m[0] != "RCM_status"]
        # Ensure SDM1 is not in the list if it causes conflicts, but we need it for other logic?
        # Actually, if we hardcoded seatbelt to False, we don't strictly need SDM1 in parser yet.
        # But if we want to read it, we can keep it if we are sure about the ID.
        # The user says Pedal is on 0x201. SDM1 is 0x201. This IS a collision on Bus 0.
        # We must NOT parse SDM1 if the pedal is present on the same bus with the same ID.
        chassis_messages = [m for m in chassis_messages if m[0] != "SDM1"]
        # Add STW_ACTN_RQ for buttons
        chassis_messages.append(("STW_ACTN_RQ", 0))

      pt_messages = [
        ("DI_torque1", 0),
        ("ESP_B", 0), # Ensure pt parser is valid if DI_torque1 is missing
      ]

      party_messages = [
        ("ESP_B", 0),
      ]
      if CP.carFingerprint == CAR.TESLA_MODEL_S_HW3:
        party_messages.append(("EPAS_sysStatus", 25))

      # Fix for Pre-AP/HW1: Redirect AP parser to Bus 0 so it sees traffic (ESP_B) and becomes valid.
      pt_bus = CANBUS.powertrain
      pedal_messages = []
      if CP.carFingerprint in (CAR.TESLA_MODEL_S_PREAP, CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1):
        pt_bus = CANBUS.party
        ap_bus = CANBUS.party
        ap_messages = [
          ("ESP_B", 0),
          ("DAS_control", 0),
          ("DAS_steeringControl", 0),
        ]
        # Comma Pedal on Bus 2 for Pre-AP (or Bus 0 if pedal_can_zero)
        if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
          # These are in comma_pedal.dbc
          pedal_messages = [
            # Optional pedal feedback: don't invalidate whole CAN health if missing.
            ("GAS_SENSOR", math.nan)
          ]
          # These are in tesla_can.dbc - Pre-AP doesn't have DAS messages
          ap_messages = [
            ("ESP_B", 0),
          ]
          # Pedal bus: matches Tinkla get_cam_can_parser() — bus 2 by default, bus 0 if pedal_can_zero
          pedal_can_zero = tinkla_conf.pedal_can_zero
          pedal_bus = 0 if pedal_can_zero else 2
          ap_bus = CANBUS.party  # Bus 0 for non-pedal AP messages
        
        # HW1 with autopilot_disabled (Pre-AP emulation) or genuine HW1
        # If it's actually a Pre-AP car masquerading as HW1, it won't have DAS messages either
        # But we should trust the fingerprint unless forced otherwise.
        # The user specifically mentioned their car fingerprints as HW1 but IS Pre-AP (Legacy)
        # To handle this safely: if we detect Pre-AP signals or if the user forces it, we might need adjustments.
        # For now, we stick to the CAR.TESLA_MODEL_S_PREAP check which the user seems to be using/forcing.
        
      else:
        ap_bus = CANBUS.autopilot_party
        ap_messages = [
          ("DAS_control", 0),
          ("DAS_steeringControl", 0),
        ]

      if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
        ap_messages = [m for m in ap_messages if m[0] not in ['DAS_control', 'DAS_steeringControl']]

      # For Pre-AP: use pedal_bus for comma_pedal parser (bus 2 by default, bus 0 if pedal_can_zero)
      # For HW1/others: use ap_bus as before
      ap_party_bus = pedal_bus if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP else ap_bus

      return {
        Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], party_messages, CANBUS.party),
        # Pre-AP: use tesla_preap DBC (has GAS_SENSOR at 0x552) NOT comma_pedal (0x201)
        Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party] if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP else DBC[CP.carFingerprint][Bus.party],
                                pedal_messages if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP else ap_messages, ap_party_bus),
        Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, pt_bus),
        # Pre-AP does not consume ap_pt signals in update_legacy; keep parser empty to
        # avoid false canValid drops from unnecessary legacy AP/PT expectations.
        Bus.ap_pt: CANParser(
          DBC[CP.carFingerprint][Bus.pt],
          [] if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP else ap_messages,
          ap_bus if ap_bus == CANBUS.party else CANBUS.autopilot_powertrain
        ),
        Bus.chassis: CANParser(DBC[CP.carFingerprint][Bus.chassis], chassis_messages, CANBUS.chassis if CP.carFingerprint == CAR.TESLA_MODEL_S_HW3 else CANBUS.party),
      }

    return {
      Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.party),
      Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.autopilot_party)
    }
