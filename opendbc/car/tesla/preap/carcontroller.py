import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.tesla.preap.nap_conf import nap_conf, PEDAL_DI_MIN, PEDAL_DI_ZERO
from opendbc.car.tesla.pedal.controller import compute_pedal_command
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.car.tesla.values import CANBUS, CruiseButtons
from opendbc.car.carlog import carlog


def init_preap_can(dbc_names, packers):
  packers[CANBUS.autopilot_party] = CANPacker(dbc_names[Bus.party])
  tesla_can = TeslaCANPreAP(packers)
  tesla_can.pedal_can_bus = nap_conf.pedal_can_bus
  return tesla_can


class PreAPLongController:

  def __init__(self):
    self.prev_pedal_di = 0.0
    self.prev_enable_long_control = False
    self.prev_requested_long = False
    self.preap_cancel_pending = False
    self.preap_engage_pending = False
    self.prev_preap_long_active = False
    self.preap_long_engage_frame = -1000000

  def update(self, CC, CS, frame, tesla_can, can_bus_party):
    can_sends = []
    actuators = CC.actuators

    requested_long = CS.cruiseEnabled and CS.enableLongControl
    long_active = requested_long and CC.longActive
    use_pedal = nap_conf.use_pedal
    pedal_factor = float(nap_conf.pedal_factor)
    pedal_transform_valid = np.isfinite(pedal_factor) and abs(pedal_factor) > 1e-6
    pedal_long_allowed = use_pedal and pedal_transform_valid

    if long_active and not self.prev_preap_long_active:
      self.preap_long_engage_frame = frame
      self.prev_pedal_di = 0.0

    # Cancel stock CC on ANY stalk interaction when pedal is installed.
    # Stock "dumb cruise" must never engage over the pedal.
    if pedal_long_allowed:
      if (not self.prev_requested_long) and requested_long:
        self.preap_cancel_pending = True
      if self.prev_requested_long and (not requested_long):
        self.preap_cancel_pending = True

      if CS.cruise_buttons != CS.prev_cruise_buttons and CS.cruise_buttons != CruiseButtons.IDLE:
        self.preap_cancel_pending = True

    if self.preap_cancel_pending and frame % 10 == 0:
      msg_stw = CS.msg_stw_actn_req
      if msg_stw is not None:
        stlk_counter = (int(msg_stw.get('MC_STW_ACTN_RQ', 0)) + 1) % 16
        can_sends.insert(0, tesla_can.create_action_request(
          CruiseButtons.CANCEL, can_bus_party, stlk_counter, msg_stw))
        self.preap_cancel_pending = False
    elif self.preap_engage_pending and frame % 10 == 0:
      msg_stw = CS.msg_stw_actn_req
      if msg_stw is not None:
        stlk_counter = (int(msg_stw.get('MC_STW_ACTN_RQ', 0)) + 1) % 16
        can_sends.insert(0, tesla_can.create_action_request(
          CruiseButtons.RES_ACCEL, can_bus_party, stlk_counter, msg_stw))
        self.preap_engage_pending = False

    self.prev_requested_long = requested_long

    # Non-pedal CC commands: consume flags set by engagement FSM
    if not pedal_long_allowed:
      if CS.preap_cc_cancel_needed:
        self.preap_cancel_pending = True
        CS.preap_cc_cancel_needed = False
      if CS.preap_cc_engage_needed:
        self.preap_engage_pending = True
        CS.preap_cc_engage_needed = False

    # Gate pedal sends on availability — a full TX queue on a dead bus
    # blocks USB sendcan for ALL buses including bus 0 steering.
    pedal_responding = not CS.pedal_timeout

    if frame % 2 == 0:
      self.prev_enable_long_control = CS.enableLongControl

      if long_active and pedal_long_allowed:
        try:
          if CS.out.gasPressed:
            can_sends.append(tesla_can.create_pedal_command(0, enable=0))
          else:
            accel_request = float(actuators.accel)
            target_speed_kph = float(CS.pedal_speed_kph)
            pedal_cmd, self.prev_pedal_di = compute_pedal_command(
              accel_request, CS.out.vEgo, self.prev_pedal_di, target_speed_kph)
            can_sends.append(tesla_can.create_pedal_command(pedal_cmd, enable=1))

            # Max regen warning (suppress for 2s after engage to avoid false triggers)
            engage_elapsed = (frame - self.preap_long_engage_frame) * 0.01
            if self.prev_pedal_di <= 0.95 * PEDAL_DI_MIN and engage_elapsed > 2.0:
              CS.pccEvent = "pedalMaxRegen"
            else:
              CS.pccEvent = None
        except Exception:
          carlog.exception("Pre-AP pedal command failed; sending disabled")
          idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
          can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
          self.prev_pedal_di = 0.0

      elif use_pedal and not pedal_transform_valid:
        idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
        can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
        self.prev_pedal_di = 0.0

      else:
        if use_pedal:
          idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
          if pedal_responding:
            can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
          elif frame % 100 == 0:
            # Low-rate wake pulse for unresponsive pedal (avoids flooding dead bus)
            can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
        self.prev_pedal_di = 0.0

    self.prev_preap_long_active = long_active
    return can_sends

  def send_cancel(self, CS, tesla_can):
    if not CS.pedal_timeout:
      idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
      return [tesla_can.create_pedal_command(idle_pedal, enable=0)]
    return []
