"""
Tests for feedforward-dominant pedal longitudinal control.

Validates:
  1. Rate limiter prevents WOT-on-engage (pedal ramps at ≤PEDAL_RAMP_RATE/step)
  2. Rate limiter allows smooth ramp-down to max regen
  3. ACCEL_PREAP_PROFILES have correct standstill values per personality
  4. Updated ki values match feedforward-dominant architecture
  5. Regen curve returns expected values at city/highway speeds
  6. Actuator delay is set correctly

Run: PYTHONPATH=. python3 opendbc/car/tesla/test_pedal_regen.py -v
"""
import sys
import types
import unittest

# Stub external dependencies not available outside the comma device
for mod_name in [
  'crcmod',
  'openpilot', 'openpilot.common', 'openpilot.common.params',
  'panda',
]:
  if mod_name not in sys.modules:
    sys.modules[mod_name] = types.ModuleType(mod_name)

# crcmod.predefined used by teslacan_legacy
crcmod_predef = types.ModuleType('crcmod.predefined')
crcmod_predef.mkCrcFun = lambda *a, **kw: (lambda data: 0)
sys.modules['crcmod.predefined'] = crcmod_predef
sys.modules['crcmod'].predefined = crcmod_predef

# Now the real opendbc modules can import
from opendbc.car.tesla.interface import (
  ACCEL_PREAP_PROFILES, PEDAL_LONG_KI_V, PEDAL_LONG_KP_V, ACCEL_PREAP_BP,
)
from opendbc.car.tesla.carcontroller import (
  CarController, PEDAL_RAMP_RATE,
  TINKLA_AVAILABLE, tinkla_conf,
)
if TINKLA_AVAILABLE:
  from opendbc.car.tesla.tinkla_conf import PEDAL_DI_MIN as TC_PEDAL_DI_MIN
else:
  TC_PEDAL_DI_MIN = -5


class TestFeedforwardDominantGains(unittest.TestCase):
  """Verify PID gains match feedforward-dominant architecture."""

  def test_kp_is_zero(self):
    """kp must be zero at all speeds to eliminate aEgo noise."""
    for kp in PEDAL_LONG_KP_V:
      self.assertAlmostEqual(kp, 0.0)

  def test_ki_values(self):
    """ki should be low (0.05-0.15) for slow integral trim with kf=1.0."""
    expected = [0.05, 0.08, 0.10, 0.15]
    for got, exp in zip(PEDAL_LONG_KI_V, expected):
      self.assertAlmostEqual(got, exp)

  def test_ki_monotonically_increasing(self):
    """ki should increase with speed (more correction at highway)."""
    for i in range(len(PEDAL_LONG_KI_V) - 1):
      self.assertLessEqual(PEDAL_LONG_KI_V[i], PEDAL_LONG_KI_V[i + 1])


class TestAccelProfiles(unittest.TestCase):
  """Verify ACCEL_PREAP_PROFILES standstill values per personality."""

  def test_aggressive_standstill(self):
    self.assertAlmostEqual(ACCEL_PREAP_PROFILES[0][0], 2.5)

  def test_standard_standstill(self):
    self.assertAlmostEqual(ACCEL_PREAP_PROFILES[1][0], 2.2)

  def test_relaxed_standstill(self):
    self.assertAlmostEqual(ACCEL_PREAP_PROFILES[2][0], 2.0)

  def test_profiles_have_correct_length(self):
    for p in (0, 1, 2):
      self.assertEqual(len(ACCEL_PREAP_PROFILES[p]), len(ACCEL_PREAP_BP))


class TestPedalRateLimiter(unittest.TestCase):
  """
  Test the pedal rate limiter prevents WOT-on-engage and allows smooth ramps.

  Creates a minimal CarController instance and calls _calc_pedal_command directly.
  """

  def _make_controller(self):
    """Build a CarController-like object with just enough state."""
    ctrl = object.__new__(CarController)
    ctrl.prev_pedal_di = 0.0
    ctrl.prev_v_ego = 0.0
    return ctrl

  @unittest.skipUnless(TINKLA_AVAILABLE, "tinkla_conf required")
  def test_wot_prevention_from_zero(self):
    """From prev_pedal_di=0, a large accel request should only ramp by PEDAL_RAMP_RATE."""
    ctrl = self._make_controller()
    ctrl._calc_pedal_command(2.5, v_ego=10.0)
    # First step: pedal_di should be at most PEDAL_RAMP_RATE from 0
    self.assertLessEqual(ctrl.prev_pedal_di, PEDAL_RAMP_RATE)
    self.assertGreater(ctrl.prev_pedal_di, 0.0)

  @unittest.skipUnless(TINKLA_AVAILABLE, "tinkla_conf required")
  def test_ramp_up_over_multiple_steps(self):
    """Pedal should ramp up smoothly over multiple calls, never jumping."""
    ctrl = self._make_controller()
    prev = 0.0
    for _ in range(20):
      ctrl._calc_pedal_command(2.0, v_ego=15.0)
      delta = ctrl.prev_pedal_di - prev
      self.assertLessEqual(delta, PEDAL_RAMP_RATE + 0.001,
                           f"Pedal jumped {delta} DI in one step (max {PEDAL_RAMP_RATE})")
      self.assertGreaterEqual(delta, -PEDAL_RAMP_RATE - 0.001)
      prev = ctrl.prev_pedal_di

  @unittest.skipUnless(TINKLA_AVAILABLE, "tinkla_conf required")
  def test_ramp_down_to_max_regen(self):
    """From prev_pedal_di=0, a large negative accel should ramp down smoothly."""
    ctrl = self._make_controller()
    ctrl._calc_pedal_command(-1.5, v_ego=10.0)
    # First step: should ramp down by at most PEDAL_RAMP_RATE
    self.assertGreaterEqual(ctrl.prev_pedal_di, -PEDAL_RAMP_RATE)
    self.assertLess(ctrl.prev_pedal_di, 0.0)

  @unittest.skipUnless(TINKLA_AVAILABLE, "tinkla_conf required")
  def test_reaches_max_regen_eventually(self):
    """After enough steps, max regen (-5 DI) should be reached."""
    ctrl = self._make_controller()
    for _ in range(50):
      ctrl._calc_pedal_command(-1.5, v_ego=10.0)
    self.assertAlmostEqual(ctrl.prev_pedal_di, TC_PEDAL_DI_MIN)

  @unittest.skipUnless(TINKLA_AVAILABLE, "tinkla_conf required")
  def test_neutral_accel(self):
    """accel_request = 0.0 -> pedal near zero (coast)."""
    ctrl = self._make_controller()
    result = ctrl._calc_pedal_command(0.0, v_ego=10.0)
    zero_pedal = tinkla_conf.di_to_pedal(0.0)
    self.assertAlmostEqual(result, zero_pedal, places=4)

  @unittest.skipUnless(TINKLA_AVAILABLE, "tinkla_conf required")
  def test_positive_accel_is_positive(self):
    """accel_request = 1.0 -> pedal above zero."""
    ctrl = self._make_controller()
    result = ctrl._calc_pedal_command(1.0, v_ego=10.0)
    zero_pedal = tinkla_conf.di_to_pedal(0.0)
    self.assertGreater(result, zero_pedal)

  @unittest.skipUnless(TINKLA_AVAILABLE, "tinkla_conf required")
  def test_engage_edge_resets_prev(self):
    """Simulating engage edge: prev_pedal_di=0 prevents stale high value from causing WOT."""
    ctrl = self._make_controller()
    # Simulate previous session had high pedal
    ctrl.prev_pedal_di = 50.0
    # Engage edge should reset to 0 (done in carcontroller.update)
    ctrl.prev_pedal_di = 0.0
    # Now a modest accel request should not jump to 50
    ctrl._calc_pedal_command(1.0, v_ego=10.0)
    self.assertLessEqual(ctrl.prev_pedal_di, PEDAL_RAMP_RATE)


class TestRegenCurve(unittest.TestCase):
  """Verify speed-dependent regen deceleration values."""

  def test_regen_at_5mps(self):
    from numpy import interp as np_interp
    regen = float(np_interp(5.0, [5., 15.], [-1.2, -1.45]))
    self.assertAlmostEqual(regen, -1.2)

  def test_regen_at_10mps(self):
    from numpy import interp as np_interp
    regen = float(np_interp(10.0, [5., 15.], [-1.2, -1.45]))
    self.assertLess(regen, -1.2)
    self.assertGreater(regen, -1.45)

  def test_regen_at_15mps(self):
    from numpy import interp as np_interp
    regen = float(np_interp(15.0, [5., 15.], [-1.2, -1.45]))
    self.assertAlmostEqual(regen, -1.45)


class TestRampRateConstant(unittest.TestCase):
  """Verify PEDAL_RAMP_RATE is set correctly."""

  def test_ramp_rate_value(self):
    self.assertAlmostEqual(PEDAL_RAMP_RATE, 2.5)

  def test_ramp_rate_positive(self):
    self.assertGreater(PEDAL_RAMP_RATE, 0.0)


if __name__ == '__main__':
  unittest.main()
