#!/usr/bin/env python3
"""Packed-CAN Tesla MCU map speed source for Pre-AP CarStateSP.speedLimit."""
import unittest

from opendbc.can import CANPacker
from opendbc.car import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CruiseButtons


class TestPreAPMapSpeed(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker("tesla_preap")

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

  def _chassis(self, extra):
    messages = [
      ("ESP_B", {}),
      ("BrakeMessage", {}),
      ("DI_state", {"DI_speedUnits": 1}),
      ("DI_torque2", {"DI_gear": 4}),
      ("DI_torque1", {}),
      ("GTW_carState", {}),
      ("STW_ANGLHP_STAT", {}),
      ("EPAS_sysStatus", {"EPAS_handsOnLevel": 0, "EPAS_eacStatus": 1, "EPAS_eacErrorCode": 0}),
      ("STW_ACTN_RQ", {"SpdCtrlLvr_Stat": CruiseButtons.IDLE}),
      ("SDM1", {"SDM_bcklDrivStatus": 1}),
    ]
    messages.extend(extra)
    return messages

  def _speed_limit(self, extra, t=1):
    CI = self._make_interface()
    _, CS_SP = CI.update(self._frames(self._chassis(extra), t=t))
    return CS_SP.speedLimit

  def test_missing_map_source_is_zero(self):
    self.assertEqual(self._speed_limit([]), 0.0)

  def test_mpp_mph_converts_to_ms(self):
    limit = self._speed_limit([
      ("UI_gpsVehicleSpeed", {"UI_mppSpeedLimit": 25, "UI_mapSpeedLimitUnits": 0}),
    ])
    self.assertAlmostEqual(limit, 25 * CV.MPH_TO_MS, places=5)

  def test_mpp_kph_uses_mcu_unit_not_di(self):
    limit = self._speed_limit([
      ("UI_gpsVehicleSpeed", {"UI_mppSpeedLimit": 40, "UI_mapSpeedLimitUnits": 1}),
    ])
    self.assertAlmostEqual(limit, 40 * CV.KPH_TO_MS, places=5)
    self.assertNotAlmostEqual(limit, 40 * CV.MPH_TO_MS, places=5)

  def test_zero_mpp_is_not_a_fabricated_limit(self):
    limit = self._speed_limit([
      ("UI_gpsVehicleSpeed", {"UI_mppSpeedLimit": 0, "UI_mapSpeedLimitUnits": 0}),
      ("UI_driverAssistMapData", {"UI_mapSpeedLimit": 13}),
    ])
    self.assertEqual(limit, 0.0)

  def test_unknown_sna_unlimited_types_are_zero(self):
    for map_type in (0, 30, 31):
      with self.subTest(map_type=map_type):
        limit = self._speed_limit([
          ("UI_gpsVehicleSpeed", {"UI_mppSpeedLimit": 40, "UI_mapSpeedLimitUnits": 0}),
          ("UI_driverAssistMapData", {"UI_mapSpeedLimit": map_type}),
        ])
        self.assertEqual(limit, 0.0)

  def test_valid_type_does_not_fabricate_from_enum(self):
    limit = self._speed_limit([
      ("UI_gpsVehicleSpeed", {"UI_mppSpeedLimit": 25, "UI_mapSpeedLimitUnits": 0}),
      ("UI_driverAssistMapData", {"UI_mapSpeedLimit": 13}),
    ])
    self.assertAlmostEqual(limit, 25 * CV.MPH_TO_MS, places=5)

  def test_stale_mpp_clears_without_killing_can_valid(self):
    CI = self._make_interface()
    packets = self._frames(self._chassis([
      ("UI_gpsVehicleSpeed", {"UI_mppSpeedLimit": 40, "UI_mapSpeedLimitUnits": 0}),
    ]), t=1)
    CS, CS_SP = CI.update(packets)
    self.assertAlmostEqual(CS_SP.speedLimit, 40 * CV.MPH_TO_MS, places=5)
    self.assertTrue(CS.canValid)
    CS, CS_SP = CI.update(self._frames(self._chassis([]), t=1 + int(11e9)))
    self.assertEqual(CS_SP.speedLimit, 0.0)
    self.assertTrue(CS.canValid)

  def test_future_mpp_timestamp_clears_without_killing_can_valid(self):
    CI = self._make_interface()
    CS, CS_SP = CI.update(self._frames(self._chassis([
      ("UI_gpsVehicleSpeed", {"UI_mppSpeedLimit": 40, "UI_mapSpeedLimitUnits": 0}),
    ]), t=int(20e9)))
    self.assertAlmostEqual(CS_SP.speedLimit, 40 * CV.MPH_TO_MS, places=5)
    CS, CS_SP = CI.update(self._frames(self._chassis([]), t=1))
    self.assertEqual(CS_SP.speedLimit, 0.0)
    self.assertTrue(CS.canValid)

  def test_muxed_road_sign_is_not_used(self):
    limit = self._speed_limit([
      ("UI_driverAssistRoadSign", {"UI_roadSign": 3, "UI_baseMapSpeedLimitMPS": 11.0}),
    ])
    self.assertEqual(limit, 0.0)


if __name__ == "__main__":
  unittest.main()
