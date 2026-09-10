#!/usr/bin/env python3
"""Packed-CAN seatbelt, pedal coexistence, and hands-on level tests for Pre-AP."""
import unittest
from unittest.mock import PropertyMock, patch

from opendbc.can import CANPacker
from opendbc.car import CanData, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.tesla.preap.nap_conf import nap_conf
from opendbc.car.tesla.values import CruiseButtons
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

ForceDisable = structs.CarStateSP.PreapLateralIntent.forceDisable


class TestPreAPSeatbeltAndHandsOn(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker("tesla_preap")
    self._millis = 1000

  def _make_interface(self):
    CarInterface = interfaces["TESLA_MODEL_S_PREAP"]
    CP = CarInterface.get_params("TESLA_MODEL_S_PREAP",
                                 {i: {} for i in range(8)},
                                 [],
                                 alpha_long=False, is_release=False, docs=False)
    return CarInterface(CP)

  def _frames(self, messages, t=1, bus=0):
    frames = []
    for name, values in messages:
      address, dat, src = self.packer.make_can_msg(name, bus, values)
      frames.append(CanData(address, dat, src))
    return [(t, frames)]

  def _chassis(self, extra, stalk=CruiseButtons.IDLE, gear=4, hands=0):
    messages = [
      ("ESP_B", {}),
      ("BrakeMessage", {}),
      ("DI_state", {}),
      ("DI_torque2", {"DI_gear": gear}),
      ("DI_torque1", {}),
      ("GTW_carState", {}),
      ("STW_ANGLHP_STAT", {}),
      ("EPAS_sysStatus", {
        "EPAS_handsOnLevel": hands,
        "EPAS_eacStatus": 1,
        "EPAS_eacErrorCode": 0,
      }),
      ("STW_ACTN_RQ", {"SpdCtrlLvr_Stat": stalk}),
    ]
    messages.extend(extra)
    return messages

  def _update(self, CI, extra, stalk=CruiseButtons.IDLE, gear=4, hands=0, t=1, bus=0):
    CS, CS_SP = CI.update(self._frames(self._chassis(extra, stalk=stalk, gear=gear, hands=hands), t=t, bus=bus))
    return CS, CS_SP

  def _double_pull(self, CI, extra, gear=4):
    def now():
      self._millis += 50
      return self._millis

    with patch("opendbc.car.tesla.preap.carstate._current_time_millis", side_effect=now):
      self._update(CI, extra, stalk=CruiseButtons.IDLE, gear=gear)
      self._update(CI, extra, stalk=CruiseButtons.MAIN, gear=gear)
      self._update(CI, extra, stalk=CruiseButtons.IDLE, gear=gear)
      return self._update(CI, extra, stalk=CruiseButtons.MAIN, gear=gear)

  def test_missing_sdm1_is_unlatched_and_does_not_poison_can_valid(self):
    CI = self._make_interface()
    CS, _ = self._update(CI, extra=[])
    self.assertTrue(CS.seatbeltUnlatched)
    self.assertTrue(CS.canValid)

  def test_status_one_is_latched(self):
    CI = self._make_interface()
    CS, _ = self._update(CI, extra=[("SDM1", {"SDM_bcklDrivStatus": 1})])
    self.assertFalse(CS.seatbeltUnlatched)

  def test_status_not_one_is_unlatched(self):
    CI = self._make_interface()
    for status in (0, 2, 3):
      with self.subTest(status=status):
        CS, _ = self._update(CI, extra=[("SDM1", {"SDM_bcklDrivStatus": status})])
        self.assertTrue(CS.seatbeltUnlatched)

  def test_stale_sdm1_is_unlatched(self):
    CI = self._make_interface()
    CS, _ = self._update(CI, extra=[("SDM1", {"SDM_bcklDrivStatus": 1})], t=1)
    self.assertFalse(CS.seatbeltUnlatched)
    CS, _ = CI.update([(1 + int(2e9), [])])
    self.assertTrue(CS.seatbeltUnlatched)

  def test_sdm1_stale_while_other_can_continues_is_unlatched(self):
    CI = self._make_interface()
    CS, _ = self._update(CI, extra=[("SDM1", {"SDM_bcklDrivStatus": 1})], t=1)
    self.assertFalse(CS.seatbeltUnlatched)
    CS, _ = self._update(CI, extra=[], t=1 + int(2e9))
    self.assertTrue(CS.seatbeltUnlatched)
    self.assertTrue(CS.canValid)

  def test_future_sdm1_timestamp_is_unlatched(self):
    CI = self._make_interface()
    CS, _ = self._update(CI, extra=[("SDM1", {"SDM_bcklDrivStatus": 1})], t=int(2e9))
    self.assertFalse(CS.seatbeltUnlatched)
    CS, _ = self._update(CI, extra=[], t=1)
    self.assertTrue(CS.seatbeltUnlatched)
    self.assertTrue(CS.canValid)

  def test_unlatched_refuses_engagement(self):
    CI = self._make_interface()
    CS, _ = self._double_pull(CI, extra=[("SDM1", {"SDM_bcklDrivStatus": 0})])
    self.assertTrue(CS.seatbeltUnlatched)
    self.assertFalse(CS.cruiseState.enabled)
    self.assertFalse(CI.CS.engagement.cruiseEnabled)

  def test_active_unlatch_tears_down_engagement_and_emits_force_disable(self):
    CI = self._make_interface()
    CS, _ = self._double_pull(CI, extra=[("SDM1", {"SDM_bcklDrivStatus": 1})])
    self.assertFalse(CS.seatbeltUnlatched)
    self.assertTrue(CS.cruiseState.enabled)

    CS, CS_SP = self._update(CI, extra=[("SDM1", {"SDM_bcklDrivStatus": 0})], stalk=CruiseButtons.IDLE)
    self.assertTrue(CS.seatbeltUnlatched)
    self.assertFalse(CS.cruiseState.enabled)
    self.assertFalse(CI.CS.engagement.cruiseEnabled)
    self.assertEqual(CS_SP.preapLateralIntent, ForceDisable)

  def test_pedal_on_bus0_coexists_with_sdm1(self):
    with patch.object(type(nap_conf), "pedal_can_zero", new_callable=PropertyMock, return_value=True):
      CI = self._make_interface()
      CS, _ = self._update(CI, extra=[
        ("SDM1", {"SDM_bcklDrivStatus": 1}),
        ("GAS_SENSOR", {"STATE": 5, "INTERCEPTOR_GAS": 0.5, "INTERCEPTOR_GAS2": 0.5}),
      ], bus=0)
      self.assertFalse(CS.seatbeltUnlatched)
      self.assertEqual(CI.CS.pedal.interceptor_state, 5)

  def test_default_safety_param_disengages_at_level_two(self):
    for hands, should_disengage in ((1, False), (2, True), (3, True)):
      with self.subTest(hands=hands):
        CI = self._make_interface()
        CS, _ = self._update(CI, extra=[("SDM1", {"SDM_bcklDrivStatus": 1})], hands=hands)
        self.assertEqual(CS.steeringDisengage, should_disengage)

  def test_encoded_level_one_trips_at_one_not_zero(self):
    CI = self._make_interface()
    CI.CP.safetyConfigs[0].safetyParam = 1 << 8
    CS, _ = self._update(CI, extra=[("SDM1", {"SDM_bcklDrivStatus": 1})], hands=0)
    self.assertFalse(CS.steeringDisengage)
    CS, _ = self._update(CI, extra=[("SDM1", {"SDM_bcklDrivStatus": 1})], hands=1)
    self.assertTrue(CS.steeringDisengage)

  def test_encoded_level_three_does_not_trip_at_two(self):
    CI = self._make_interface()
    CI.CP.safetyConfigs[0].safetyParam = 3 << 8
    CS, _ = self._update(CI, extra=[("SDM1", {"SDM_bcklDrivStatus": 1})], hands=2)
    self.assertFalse(CS.steeringDisengage)
    CS, _ = self._update(CI, extra=[("SDM1", {"SDM_bcklDrivStatus": 1})], hands=3)
    self.assertTrue(CS.steeringDisengage)

  def test_pause_gate_clears_disengage_at_configured_level(self):
    CI = self._make_interface()
    CI.CP.safetyConfigs[0].safetyParam = 2 << 8
    CI.CP_SP.flags |= int(TeslaFlagsSP.PREAP_HANDS_ON_PAUSE)
    CS, _ = self._update(CI, extra=[("SDM1", {"SDM_bcklDrivStatus": 1})], hands=2)
    self.assertFalse(CS.steeringDisengage)

  def test_pause_gate_level_three_does_not_trip_at_two(self):
    CI = self._make_interface()
    CI.CP.safetyConfigs[0].safetyParam = 3 << 8
    CI.CP_SP.flags |= int(TeslaFlagsSP.PREAP_HANDS_ON_PAUSE)
    CS, _ = self._update(CI, extra=[("SDM1", {"SDM_bcklDrivStatus": 1})], hands=2)
    self.assertFalse(CS.steeringDisengage)
    CS, _ = self._update(CI, extra=[("SDM1", {"SDM_bcklDrivStatus": 1})], hands=3)
    self.assertFalse(CS.steeringDisengage)


if __name__ == "__main__":
  unittest.main()
