import copy
import math
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.values import DBC, CANBUS, GEAR_MAP, STEER_THRESHOLD, CAR, TeslaLegacyParams, LEGACY_CARS
from opendbc.car.tesla.nap_conf import nap_conf
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.preap.pedal_feedback import PedalFeedback
from opendbc.car.tesla.preap.carstate import update_preap


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
      double_pull_enabled=nap_conf.double_pull_enabled,
      double_pull_window_ms=nap_conf.double_pull_window_ms,
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
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      return update_preap(self, can_parsers)

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
    ret.gasPressed = cp_pt.vl["DI_torque1"]["DI_pedalPos"] > 0

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp_chassis.vl["BrakeMessage"]["driverBrakeStatus"] == 2

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
    ret.steerFaultTemporary = eac_status == "EAC_INHIBITED"

    # FSD disengages using union of handsOnLevel (slow overrides) and high angle rate faults (fast overrides, high speed)
    eac_error_code = self.can_defines["EPAS_sysStatus"]["EPAS_eacErrorCode"].get(int(epas_status["EPAS_eacErrorCode"]), None)
    ret.steeringDisengage = self.hands_on_level >= 3 or (eac_status == "EAC_INHIBITED" and
                                                          eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")

    # Cruise state
    cruise_state = self.can_defines["DI_state"]["DI_cruiseState"].get(int(cp_chassis.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units = self.can_defines["DI_state"]["DI_speedUnits"].get(int(cp_chassis.vl["DI_state"]["DI_speedUnits"]), None)

    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")
    ret.cruiseState.enabled = cruise_enabled
    ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled

    if speed_units is not None:
      self.speed_units = speed_units

    if speed_units == "KPH":
      ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS, 1e-3)
    elif speed_units == "MPH":
      ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS, 1e-3)

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
    else:
      ret.seatbeltUnlatched = cp_chassis.vl["SDM1"]["SDM_bcklDrivStatus"] != 1

    # AEB
    ret.stockAeb = cp_ap_pt.vl["DAS_control"]["DAS_aebEvent"] == 1

    # LKAS
    ret.stockLkas = cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"] == 2  # LANE_KEEP_ASSIST

    # Stock Autosteer should be off (includes FSD)
    # ret.invalidLkasSetting = cp_ap_party.vl["DAS_settings"]["DAS_autosteerEnabled"] != 0

    # Messages needed by carcontroller
    self.das_control = copy.copy(cp_ap_pt.vl["DAS_control"])
    self.cruise_enabled_prev = ret.cruiseState.enabled

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
          pedal_can_zero = nap_conf.pedal_can_zero
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
