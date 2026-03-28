import numpy as np

from opendbc.car import get_safety_config, structs, STD_CARGO_KG
from opendbc.car.carlog import carlog
from opendbc.car.tesla.values import TeslaSafetyFlags
from opendbc.car.tesla.preap.nap_conf import nap_conf
from opendbc.car.tesla.preap.constants import (
  ACCEL_PREAP_BP, ACCEL_PREAP_PROFILES,
  PEDAL_LONG_K_BP, PEDAL_LONG_KP_V, PEDAL_LONG_KI_V,
)

try:
  from openpilot.common.params import Params as _Params
  _params = _Params()
except ImportError:
  _params = None


def get_preap_accel_limits(current_speed):
  personality = 1
  if _params is not None:
    try:
      personality = int(_params.get("LongitudinalPersonality", return_default=True))
    except (TypeError, ValueError):
      pass
  profile = ACCEL_PREAP_PROFILES.get(personality, ACCEL_PREAP_PROFILES[1])
  a_max = float(np.interp(current_speed, ACCEL_PREAP_BP, profile))
  return -1.5, a_max


def get_preap_params(ret, fingerprint):
  flags = TeslaSafetyFlags.FLAG_PREAP | TeslaSafetyFlags.LONG_CONTROL

  use_pedal = nap_conf.use_pedal
  radar_enabled = nap_conf.radar_enabled
  radar_behind_nosecone = nap_conf.radar_behind_nosecone
  carlog.info("Pre-AP fingerprint: use_pedal=%s radar_enabled=%s behind_nosecone=%s",
              use_pedal, radar_enabled, radar_behind_nosecone)

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
  ret.openpilotLongitudinalControl = True
  ret.steerControlType = structs.CarParams.SteerControlType.angle
  ret.pcmCruise = False

  if use_pedal:
    ret.longitudinalTuning.kpBP = PEDAL_LONG_K_BP
    ret.longitudinalTuning.kpV = PEDAL_LONG_KP_V
    ret.longitudinalTuning.kiBP = PEDAL_LONG_K_BP
    ret.longitudinalTuning.kiV = PEDAL_LONG_KI_V
    # kf=1.0 feedforward: a_target passes through 1:1 to pedal mapping
    try:
      ret.longitudinalTuning.kf = 1.0
    except AttributeError:
      pass  # kf not available in all capnp schema versions
    ret.longitudinalActuatorDelay = 0.3
  else:
    ret.longitudinalTuning.kpBP = [0.0]
    ret.longitudinalTuning.kpV = [0.0]
    ret.longitudinalTuning.kiBP = [0.0]
    ret.longitudinalTuning.kiV = [0.0]

  # Legacy Model S steering and physical params
  ret.steerLimitTimer = 0.4
  ret.steerActuatorDelay = 0.1
  ret.steerAtStandstill = True
  ret.alphaLongitudinalAvailable = False
  ret.vEgoStopping = 0.1
  ret.vEgoStarting = 0.1
  ret.stoppingDecelRate = 1.0

  ret.mass = 2100. + STD_CARGO_KG
  ret.wheelbase = 2.959
  ret.centerToFront = ret.wheelbase * 0.5
  ret.steerRatio = 15.0

  return ret
