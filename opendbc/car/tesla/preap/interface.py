"""
Pre-AP CarParams configuration and accel limits.

Extracted from interface.py so upstream changes to _get_params() and
get_pid_accel_limits() don't conflict with Pre-AP logic.
"""
import numpy as np

from opendbc.car import get_safety_config, structs, STD_CARGO_KG
from opendbc.car.tesla.values import TeslaSafetyFlags
from opendbc.car.tesla.preap.nap_conf import nap_conf
from opendbc.car.tesla.preap.constants import (
  ACCEL_PREAP_BP, ACCEL_PREAP_PROFILES,
  PEDAL_LONG_K_BP, PEDAL_LONG_KP_V, PEDAL_LONG_KI_V,
)

# Read openpilot Params for personality toggle (may fail outside device)
try:
  from openpilot.common.params import Params as _Params
  _params = _Params()
except ImportError:
  _params = None


def get_preap_accel_limits(current_speed):
  """Pre-AP pedal accel envelope based on Driving Personality toggle.

  Returns (a_min, a_max) in m/s².
  """
  personality = 1  # default to standard
  if _params is not None:
    try:
      personality = int(_params.get("LongitudinalPersonality", return_default=True))
    except (TypeError, ValueError):
      pass
  profile = ACCEL_PREAP_PROFILES.get(personality, ACCEL_PREAP_PROFILES[1])
  a_max = float(np.interp(current_speed, ACCEL_PREAP_BP, profile))
  return -1.5, a_max


def get_preap_params(ret, fingerprint):
  """Configure CarParams for Pre-AP Model S.

  Args:
    ret: structs.CarParams builder
    fingerprint: dict of fingerprints per bus

  Returns:
    Modified ret with Pre-AP configuration applied.
  """
  flags = TeslaSafetyFlags.FLAG_PREAP | TeslaSafetyFlags.LONG_CONTROL

  use_pedal = nap_conf.use_pedal
  radar_enabled = nap_conf.radar_enabled
  radar_behind_nosecone = nap_conf.radar_behind_nosecone
  print(f"[NAP] interface.py fingerprint: use_pedal={use_pedal}, "
        f"radar_enabled={radar_enabled}, radar_behind_nosecone={radar_behind_nosecone}, "
        f"radarUnavailable={not radar_enabled}")

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
  ret.pcmCruise = False  # We control engagement manually

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
    # Actuator delay: comma pedal CAN + drivetrain response.
    # Real chain ~250-350ms (CAN→pedal→inverter→drivetrain).
    # 0.5s was too high — MPC over-predicted, making the car feel sluggish.
    ret.longitudinalActuatorDelay = 0.3
  else:
    ret.longitudinalTuning.kpBP = [0.0]
    ret.longitudinalTuning.kpV = [0.0]
    ret.longitudinalTuning.kiBP = [0.0]
    ret.longitudinalTuning.kiV = [0.0]

  # Shared legacy params (duplicated here so _get_params_sx can early-return
  # for Pre-AP, eliminating Pre-AP as a merge conflict surface)
  ret.steerLimitTimer = 0.4
  ret.steerActuatorDelay = 0.1
  ret.steerAtStandstill = True
  ret.alphaLongitudinalAvailable = False
  ret.vEgoStopping = 0.1
  ret.vEgoStarting = 0.1
  # Tinkla uses a stronger stopping decel ramp for Pre-AP.
  ret.stoppingDecelRate = 1.0

  # Set physical params explicitly to avoid 0.0 ratio error
  ret.mass = 2100. + STD_CARGO_KG
  ret.wheelbase = 2.959
  ret.centerToFront = ret.wheelbase * 0.5
  ret.steerRatio = 15.0

  return ret
