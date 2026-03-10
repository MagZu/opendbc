#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.tesla.values import CruiseButtons, TeslaSafetyFlags
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py


class TestTeslaPreAPStalkRearm(unittest.TestCase):
  TX_MSGS = [[0x488, 0], [0x2B9, 0], [0x214, 0], [0x551, 0], [0x551, 2], [0x45, 0], [0x659, 0]]

  def setUp(self):
    self.safety = libsafety_py.libsafety
    flags = int(TeslaSafetyFlags.LONG_CONTROL | TeslaSafetyFlags.FLAG_PREAP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaLegacy, flags)
    self.safety.init_tests()
    self.packer = CANPackerSafety("tesla_preap")

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _stalk_msg(self, lever_position):
    return self.packer.make_can_msg_safety("STW_ACTN_RQ", 0, {"SpdCtrlLvr_Stat": lever_position})

  def _epas_msg(self, hands_on_level, eac_status=1, eac_error_code=0):
    values = {
      "EPAS_handsOnLevel": hands_on_level,
      "EPAS_eacStatus": eac_status,
      "EPAS_eacErrorCode": eac_error_code,
      "EPAS_internalSAS": 0,
    }
    return self.packer.make_can_msg_safety("EPAS_sysStatus", 0, values)

  def _gear_msg(self, gear):
    return self.packer.make_can_msg_safety("DI_torque2", 0, {"DI_gear": gear})

  def _door_msg_closed(self):
    return self.packer.make_can_msg_safety("GTW_carState", 0, {})

  def test_stalk_reengages_after_steering_disengage(self):
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_cruise_engaged_prev())

    # Keep engage preconditions explicitly valid.
    self.assertTrue(self._rx(self._gear_msg(4)))
    self.assertTrue(self._rx(self._door_msg_closed()))

    # Initial stalk pull engages controls.
    self.assertTrue(self._rx(self._stalk_msg(CruiseButtons.MAIN)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_cruise_engaged_prev())

    # Steering disengage must drop both controls and cruise edge state.
    self.assertTrue(self._rx(self._epas_msg(hands_on_level=3)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_cruise_engaged_prev())

    # Clear EPAS override signal.
    self.assertTrue(self._rx(self._epas_msg(hands_on_level=0)))
    self.assertFalse(self.safety.get_controls_allowed())

    # Next stalk pull must re-arm controls.
    self.assertTrue(self._rx(self._stalk_msg(CruiseButtons.MAIN)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_cruise_engaged_prev())


if __name__ == "__main__":
  unittest.main()
