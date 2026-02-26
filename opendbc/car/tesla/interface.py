import numpy as np

from opendbc.car import get_safety_config, structs, STD_CARGO_KG
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import TeslaSafetyFlags, CAR, TeslaLegacyParams, LEGACY_CARS, CruiseButtons
from opendbc.car.tesla.radar_interface import RadarInterface

# Import config helper - may fail on non-comma devices during testing
try:
  from opendbc.car.tesla.tinkla_conf import tinkla_conf
except ImportError:
  tinkla_conf = None

# Read openpilot Params for personality toggle (may fail outside device)
try:
  from openpilot.common.params import Params as _Params
  _params = _Params()
except ImportError:
  _params = None

# Pre-AP pedal accel envelopes, mapped to openpilot Driving Personality toggle.
# Breakpoints are speed in m/s; values are max accel in m/s².
#   aggressive(0) → sporty response, ~80kW available at highway
#   standard(1)   → balanced daily driver
#   relaxed(2)    → smooth and gentle, still usable (old "Chill" was too weak)
ACCEL_PREAP_BP = [0.0, 1.3, 7.5, 15.0, 25.0, 40.0]  # m/s
ACCEL_PREAP_PROFILES = {
  0: [2.5, 2.3, 2.0, 1.5, 1.2, 1.0],   # aggressive: Tinkla AP MadMax low-speed
  1: [2.2, 2.0, 1.5, 1.2, 0.9, 0.7],   # standard: Tinkla AP Standard low-speed
  2: [2.0, 1.8, 1.2, 0.9, 0.75, 0.55], # relaxed: Tinkla AP Chill low-speed
}

# Feedforward-dominant longitudinal tune (FrogPilot/OPGM Bolt-inspired).
# kp=0: no proportional term — eliminates aEgo sensor noise amplification.
# kf=1.0: full a_target passthrough — the MPC plan (jerk-constrained, smooth)
#   is the dominant control signal.
# ki low: slow integral trim handles steady-state offset (hills, wind, regen).
#   Lower than FP Bolt (0.125-0.33) because our kf=1.0 vs their kf=0.25.
PEDAL_LONG_K_BP = [0.0, 3.0, 6.0, 35.0]
PEDAL_LONG_KP_V = [0.0, 0.0, 0.0, 0.0]
PEDAL_LONG_KI_V = [0.05, 0.08, 0.10, 0.15]


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    # Pre-AP pedal: ALWAYS return pedal-safe limits, never fall through to
    # base class (-3.5, 2.0) which causes WOT.  Accel envelope follows the
    # Driving Personality toggle (Settings → Toggles → Driving Personality).
    if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      personality = 1  # default to standard
      if _params is not None:
        try:
          personality = int(_params.get("LongitudinalPersonality", return_default=True))
        except (TypeError, ValueError):
          pass
      profile = ACCEL_PREAP_PROFILES.get(personality, ACCEL_PREAP_PROFILES[1])
      a_max = float(np.interp(current_speed, ACCEL_PREAP_BP, profile))
      return -1.5, a_max

    return CarInterfaceBase.get_pid_accel_limits(CP, current_speed, cruise_speed)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    if candidate in LEGACY_CARS:
      return CarInterface._get_params_sx(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs)

    ret.brand = "tesla"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

      ret.vEgoStopping = 0.1
      ret.vEgoStarting = 0.1
      ret.stoppingDecelRate = 0.3

    # ret.dashcamOnly = candidate in (CAR.TESLA_MODEL_X) # dashcam only, pending find invalidLkasSetting signal

    return ret

  @staticmethod
  def _get_params_sx(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "tesla"

    if not any(0x201 in f for f in fingerprint.values()):
      ret.flags |= TeslaLegacyParams.NO_SDM1.value

    if candidate == CAR.TESLA_MODEL_S_PREAP:
      flags = TeslaSafetyFlags.FLAG_PREAP | TeslaSafetyFlags.LONG_CONTROL

      # Read configuration (with safe fallbacks if config unavailable)
      use_pedal = tinkla_conf.use_pedal if tinkla_conf else False
      radar_enabled = tinkla_conf.radar_enabled if tinkla_conf else False
      radar_behind_nosecone = tinkla_conf.radar_behind_nosecone if tinkla_conf else False
      print(f"[NAP] interface.py fingerprint: tinkla_conf={'present' if tinkla_conf else 'None'}, "
            f"use_pedal={use_pedal}, radar_enabled={radar_enabled}, "
            f"radar_behind_nosecone={radar_behind_nosecone}, radarUnavailable={not radar_enabled}")

      if use_pedal:
        flags |= TeslaSafetyFlags.FLAG_ENABLE_PEDAL
      if radar_enabled:
        flags |= TeslaSafetyFlags.FLAG_RADAR_EMULATION
      if radar_behind_nosecone:
        flags |= TeslaSafetyFlags.FLAG_RADAR_BEHIND_NOSECONE

      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(flags)),
      ]
      ret.radarUnavailable = not radar_enabled
      # Force longitudinal control true for Pre-AP
      ret.openpilotLongitudinalControl = True
      ret.steerControlType = structs.CarParams.SteerControlType.angle
      ret.pcmCruise = False # We control engagement manually

      # Tinkla parity: use dedicated pedal longitudinal tune when pedal mode is enabled.
      # Without this, OP runs mostly feedforward accel at low speed, which is prone to
      # hill lag/overshoot on Pre-AP pedal cars.
      if use_pedal:
        ret.longitudinalTuning.kpBP = PEDAL_LONG_K_BP
        ret.longitudinalTuning.kpV = PEDAL_LONG_KP_V
        ret.longitudinalTuning.kiBP = PEDAL_LONG_K_BP
        ret.longitudinalTuning.kiV = PEDAL_LONG_KI_V
        # Full feedforward: a_target passes through 1:1.  The MPC plan is already
        # jerk-constrained and smooth; the integrator handles residual offset.
        # Pedal rate limiter in carcontroller prevents WOT-on-engage.
        try:
          ret.longitudinalTuning.kf = 1.0
        except AttributeError:
          pass  # kf field not available in device capnp schema
        # Actuator delay: comma pedal CAN + drivetrain response.  Matches tinkla
        # (0.5s) and FrogPilot GM pedal (0.5s).  Planner looks 0.55s ahead.
        ret.longitudinalActuatorDelay = 0.5
      else:
        ret.longitudinalTuning.kpBP = [0.0]
        ret.longitudinalTuning.kpV = [0.0]
        ret.longitudinalTuning.kiBP = [0.0]
        ret.longitudinalTuning.kiV = [0.0]

      # Set physical params explicitly to avoid 0.0 ratio error
      ret.mass = 2100. + STD_CARGO_KG
      ret.wheelbase = 2.959
      ret.centerToFront = ret.wheelbase * 0.5
      ret.steerRatio = 15.0
    elif candidate in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, ):
      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW1)),
      ]
    elif candidate in (CAR.TESLA_MODEL_S_HW2,):
      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW2)),
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW2 | TeslaSafetyFlags.FLAG_EXTERNAL_PANDA)),
      ]
    elif candidate in (CAR.TESLA_MODEL_S_HW3,):
      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW3)),
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW3 | TeslaSafetyFlags.FLAG_EXTERNAL_PANDA)),
      ]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    # PREAP sets radarUnavailable from the toggle above; don't overwrite it here.
    if candidate != CAR.TESLA_MODEL_S_PREAP:
      ret.radarUnavailable = candidate in (CAR.TESLA_MODEL_S_HW2, )

    # Legacy Tesla ports in this tree run openpilot longitudinal by default
    # (not as an optional alpha toggle), so mark alpha availability false.
    ret.alphaLongitudinalAvailable = False
    ret.openpilotLongitudinalControl = True
    ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

    ret.vEgoStopping = 0.1
    ret.vEgoStarting = 0.1
    # Tinkla uses a stronger stopping decel ramp for Pre-AP.
    ret.stoppingDecelRate = 1.0 if candidate == CAR.TESLA_MODEL_S_PREAP else 0.3

    # ret.dashcamOnly = candidate in (CAR.TESLA_MODEL_X) # dashcam only, pending find invalidLkasSetting signal

    return ret

  # NOTE: Event handling is done in the main openpilot repo's controlsd/selfdrived,
  # NOT in opendbc's CarInterface. The CS.longCtrlEvent field is available for
  # the main repo to read via self.CS if needed, but we do NOT modify ret.events here.
  # The previous attempt to do so caused card to crash.
