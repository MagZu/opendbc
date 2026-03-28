"""
Pre-AP longitudinal controller and CAN init — pedal commands, cruise-over-CC,
and hardware wiring for Pre-AP Model S.

Extracted from carcontroller.py so upstream changes to the main update()
loop and __init__() don't conflict with Pre-AP logic.
"""
import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.tesla.preap.nap_conf import nap_conf, PEDAL_DI_MIN, PEDAL_DI_ZERO
from opendbc.car.tesla.pedal.controller import compute_pedal_command
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.car.tesla.values import CANBUS, CruiseButtons
from opendbc.car.carlog import carlog


def init_preap_can(dbc_names, packers):
  """Set up CAN packers and TeslaCANPreAP for Pre-AP Model S.

  Args:
    dbc_names: dict of DBC filenames per bus
    packers: dict of CANPackers (modified in place — adds autopilot_party key)

  Returns:
    TeslaCANPreAP instance configured for this car
  """
  packers[CANBUS.autopilot_party] = CANPacker(dbc_names[Bus.party])
  pedal_packer = CANPacker("comma_pedal")
  tesla_can = TeslaCANPreAP(packers, pedal_packer)
  tesla_can.pedal_can_bus = nap_conf.pedal_can_bus
  return tesla_can


class PreAPLongController:
  """Manages Pre-AP longitudinal control (pedal + cruise-over-CC).

  Owns all engagement/pedal state.  Called from CarController.update()
  on every frame for TESLA_MODEL_S_PREAP.
  """

  def __init__(self):
    self.prev_pedal_di = 0.0
    self.prev_enable_long_control = False
    self.prev_requested_long = False
    self.preap_cancel_pending = False
    self.preap_engage_pending = False
    self.prev_preap_long_active = False
    self.preap_long_engage_frame = -1000000

  def update(self, CC, CS, frame, tesla_can, can_bus_party):
    """Run one Pre-AP longitudinal cycle.

    Args:
      CC: CarControl message
      CS: CarState instance
      frame: current frame counter
      tesla_can: TeslaCANPreAP instance for building CAN messages
      can_bus_party: CANBUS.party value

    Returns:
      list of CAN messages to send
    """
    can_sends = []
    actuators = CC.actuators

    # Get engagement state (used for both pedal and pedal-over-CC)
    cs_cruise_enabled = getattr(CS, 'cruiseEnabled', False)
    cs_enable_long = getattr(CS, 'enableLongControl', False)
    requested_long = cs_cruise_enabled and cs_enable_long
    long_active = requested_long and CC.longActive
    use_pedal = nap_conf.use_pedal
    pedal_factor = float(nap_conf.pedal_factor)
    pedal_transform_valid = bool(np.isfinite(pedal_factor) and abs(pedal_factor) > 1e-6)
    pedal_long_allowed = bool(use_pedal and pedal_transform_valid)

    if long_active and not self.prev_preap_long_active:
      self.preap_long_engage_frame = frame
      self.prev_pedal_di = 0.0  # Rate limiter starts from zero on fresh engage

    # ==============================================
    # Pedal Over CC: one-shot CANCEL to keep stock CC unlatched
    # in pedal mode. Trigger on:
    #  - requested-long engage edge
    #  - requested-long disengage edge
    #  - real stalk press edges for engage/speed change
    # Do NOT use CC.cruiseControl.cancel directly here, as controlsd
    # keeps it asserted when pcmCruise is False.
    # ==============================================
    if pedal_long_allowed:
      if (not self.prev_requested_long) and requested_long and CS.out.cruiseState.enabled:
        self.preap_cancel_pending = True
      if self.prev_requested_long and (not requested_long) and CS.out.cruiseState.enabled:
        self.preap_cancel_pending = True

      cruise_buttons = getattr(CS, "cruise_buttons", CruiseButtons.IDLE)
      prev_cruise_buttons = getattr(CS, "prev_cruise_buttons", CruiseButtons.IDLE)
      stalk_press_edge = cruise_buttons != prev_cruise_buttons and cruise_buttons != CruiseButtons.IDLE
      if stalk_press_edge:
        pedal_over_cc_button = (
          cruise_buttons == CruiseButtons.MAIN
          or CruiseButtons.is_accel(cruise_buttons)
          or CruiseButtons.is_decel(cruise_buttons)
        )
        if pedal_over_cc_button and requested_long and CS.out.cruiseState.enabled:
          self.preap_cancel_pending = True

    if self.preap_cancel_pending and frame % 10 == 0:
      msg_stw = getattr(CS, 'msg_stw_actn_req', None)
      if msg_stw is not None:
        stlk_counter = (int(msg_stw.get('MC_STW_ACTN_RQ', 0)) + 1) % 16
        can_sends.insert(0, tesla_can.create_action_request(
          CruiseButtons.CANCEL, can_bus_party, stlk_counter, msg_stw))
        self.preap_cancel_pending = False
    elif self.preap_engage_pending and frame % 10 == 0:
      msg_stw = getattr(CS, 'msg_stw_actn_req', None)
      if msg_stw is not None:
        stlk_counter = (int(msg_stw.get('MC_STW_ACTN_RQ', 0)) + 1) % 16
        can_sends.insert(0, tesla_can.create_action_request(
          CruiseButtons.RES_ACCEL, can_bus_party, stlk_counter, msg_stw))
        self.preap_engage_pending = False

    self.prev_requested_long = requested_long

    # Non-pedal CC commands: consume flags set by carstate stalk handler
    if not pedal_long_allowed:
      if getattr(CS, 'preap_cc_cancel_needed', False):
        self.preap_cancel_pending = True
        CS.preap_cc_cancel_needed = False
      if getattr(CS, 'preap_cc_engage_needed', False):
        self.preap_engage_pending = True
        CS.preap_cc_engage_needed = False

    # Gate ALL pedal sends on pedal availability.
    # When pedal is not responding (unplugged or absent), sending 0x551 to a dead
    # bus fills its TX queue.  can_tx_check_min_slots_free() checks ALL bus queues,
    # so one full queue blocks USB sendcan for ALL buses — including bus 0 steering.
    # Matches tinkla: PCC_module._update_pedal_state() gates on pedal_idx changes.
    pedal_responding = not getattr(CS, 'pedal_timeout', True)

    if frame % 2 == 0:
      self.prev_enable_long_control = cs_enable_long

      if long_active and pedal_long_allowed:
        # ============================================
        # Mode 1: Comma Pedal Control
        # Matches Tinkla Pre-AP behavior: always send commands when
        # use_pedal is True and long is active. Tinkla's pcc_available
        # is always True for Pre-AP (autopilot_disabled=True).
        # ============================================
        try:
          if CS.out.gasPressed:
            # Tinkla PCC_module.py line 294: if CS.out.gasPressed, stop commanding
            # This is the SAFE approach - let the human have full control
            can_sends.append(tesla_can.create_pedal_command(0, enable=0))
          else:
            accel_request = float(actuators.accel)
            target_speed_kph = float(getattr(CS, "pedal_speed_kph", 0.0))
            pedal_cmd, self.prev_pedal_di = compute_pedal_command(
              accel_request, CS.out.vEgo, self.prev_pedal_di, target_speed_kph)
            can_sends.append(tesla_can.create_pedal_command(pedal_cmd, enable=1))

            # Max regen warning: alert driver when pedal is at/near max regen
            # (they need to use the brake pedal for more deceleration).
            # Tinkla PCC_module.py line 353: trigger at 95% of PEDAL_DI_MIN, suppress for 2s after engage.
            pedal_di_min = PEDAL_DI_MIN
            engage_elapsed = (frame - self.preap_long_engage_frame) * 0.01  # frames to seconds at 100Hz
            if self.prev_pedal_di <= 0.95 * pedal_di_min and engage_elapsed > 2.0:
              CS.pccEvent = "pedalMaxRegen"
            else:
              CS.pccEvent = None
        except Exception:
          # Fail-safe: on any unexpected pedal path exception, send disabled pedal.
          carlog.exception("Pre-AP pedal command path failed; sending disabled pedal command")
          idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
          can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
          self.prev_pedal_di = 0.0

      elif use_pedal and not pedal_transform_valid:
        # Safety gate: block pedal actuation when pedal transform is invalid.
        idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
        can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
        self.prev_pedal_di = 0.0

      else:
        # ============================================
        # Steering Only (Single Pull) or Not Engaged
        # Send idle pedal keepalive to prevent firmware fault
        # Tinkla PCC_module.py line 132: sends reset at frame % 50 (2Hz)
        # ============================================
        if use_pedal:
          if pedal_responding:
            # Pedal is alive — send idle keepalive at 50Hz (every frame %2)
            idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
            can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
          elif frame % 100 == 0:
            # Pedal not responding — send disabled reset at 1Hz to wake it up.
            # Low rate avoids flooding a dead bus (can_tx_check_min_slots_free
            # blocks ALL buses if any one queue fills).  Tinkla uses 2Hz here
            # (frame%50) but 1Hz is safer for dead-bus tolerance.
            idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
            can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
        # Reset state when not active
        self.prev_pedal_di = 0.0

    self.prev_preap_long_active = long_active
    return can_sends

  def send_cancel(self, CS, tesla_can):
    """Send idle pedal command for cancel path (when not openpilotLongitudinalControl)."""
    if not getattr(CS, 'pedal_timeout', True):
      idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
      return [tesla_can.create_pedal_command(idle_pedal, enable=0)]
    return []
