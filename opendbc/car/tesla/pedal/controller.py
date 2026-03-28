from numpy import interp, clip

from opendbc.car.tesla.preap.nap_conf import (
  nap_conf,
  PEDAL_DI_MIN, PEDAL_DI_ZERO,
  PEDAL_BP, PEDAL_MAX_VALUES,
  ACCEL_MAX,
)

# Max DI change per 20ms step (pedal sends at 50 Hz). Prevents WOT-on-engage.
# 2.5 DI/step = 125 DI/s. P85+ at highway (max=75 DI): 0→full in 0.6s.
PEDAL_RAMP_RATE = 2.5


def compute_pedal_command(accel_request: float, v_ego: float, prev_pedal_di: float,
                          target_speed_kph: float | None = None) -> tuple[float, float]:
  """Convert acceleration request (m/s²) to comma pedal voltage.

  Returns (pedal_voltage, updated_prev_pedal_di).
  """
  if nap_conf is None:
    pedal_di = float(clip(interp(accel_request, [-1.5, 0., 2.0], [-5., 0., 100.]), -5, 100))
    pedal_di = float(clip(pedal_di, prev_pedal_di - PEDAL_RAMP_RATE, prev_pedal_di + PEDAL_RAMP_RATE))
    return _fallback_di_to_pedal(pedal_di), pedal_di

  pedal_profile = nap_conf.get_pedal_profile_values()
  max_pedal_value = float(interp(v_ego, PEDAL_BP, pedal_profile))

  regen_decel = -1.5
  accel_bp = [regen_decel, 0.0, ACCEL_MAX]
  accel_v = [PEDAL_DI_MIN, 0.0, max_pedal_value]
  pedal_di = float(interp(accel_request, accel_bp, accel_v))

  pedal_di = float(clip(pedal_di, PEDAL_DI_MIN, max_pedal_value))

  # Rate limiter: cap DI change per step to prevent sudden jumps
  pedal_di = float(clip(pedal_di, prev_pedal_di - PEDAL_RAMP_RATE, prev_pedal_di + PEDAL_RAMP_RATE))

  pedal_cmd = nap_conf.di_to_pedal(pedal_di)
  return pedal_cmd, pedal_di


# Fallback constants when nap_conf unavailable
_PEDAL_CALIB_FACTOR = 1.0
_PEDAL_CALIB_ZERO = 0.0
_PEDAL_ZERO = _PEDAL_CALIB_ZERO - 1.0 / _PEDAL_CALIB_FACTOR


def _fallback_di_to_pedal(val):
  return _PEDAL_ZERO + (val - 0.0) / _PEDAL_CALIB_FACTOR
