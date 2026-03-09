from numpy import interp, clip

from opendbc.car.tesla.tinkla_conf import (
  tinkla_conf,
  PEDAL_DI_MIN, PEDAL_DI_ZERO,
  PEDAL_BP, PEDAL_V_DEFAULT,
  ACCEL_MAX,
)

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


def _fallback_di_to_pedal(val):
  """Default DI→pedal transform when tinkla_conf unavailable. Matches Tinkla tunes.py."""
  return PEDAL_ZERO_DEFAULT + (val - PEDAL_DI_ZERO_DEFAULT) / PEDAL_CALIB_FACTOR_DEFAULT


def compute_pedal_command(accel_request: float, v_ego: float, prev_pedal_di: float,
                          target_speed_kph: float | None = None) -> tuple[float, float]:
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

  Returns:
    (pedal_voltage, updated_prev_pedal_di) — caller stores the second value
    for rate limiting on the next call.
  """
  if tinkla_conf is None:
    # Fallback: simple linear mapping if tinkla_conf unavailable
    pedal_di = float(clip(interp(accel_request, [-1.5, 0., 2.0], [-5., 0., 100.]), -5, 100))
    pedal_di = float(clip(pedal_di,
                          prev_pedal_di - PEDAL_RAMP_RATE,
                          prev_pedal_di + PEDAL_RAMP_RATE))
    return _fallback_di_to_pedal(pedal_di), pedal_di

  # Trim-specific max pedal (P85+, P85, S85, S60, Generic)
  pedal_profile = tinkla_conf.get_pedal_profile_values()
  max_pedal_value = float(interp(v_ego, PEDAL_BP, pedal_profile))

  # Full regen available at all speeds (PID is already capped at -1.5 m/s²)
  regen_decel = -1.5

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
                        prev_pedal_di - PEDAL_RAMP_RATE,
                        prev_pedal_di + PEDAL_RAMP_RATE))

  # Transform DI -> pedal voltage via calibration
  pedal_cmd = tinkla_conf.di_to_pedal(pedal_di)

  return pedal_cmd, pedal_di
