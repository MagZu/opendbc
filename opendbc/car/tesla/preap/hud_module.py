"""NAP Buddy IC integration — instrument cluster rendering for Tesla Pre-AP.

Renders openpilot's view (lane path, lead car, status, warnings) on the stock
Tesla MCU1 instrument cluster. The NAP Buddy bridge sniffs a set of DAS_* frames
off chassis bus 0 and forwards them to the IC bus; the Tesla gateway does not
forward them there itself, so the bridge hardware is required.

Display-only by construction. Every frame emitted here is state the cluster
draws: lane geometry, lead-car position, ACC/AP status text and warning bits.
Nothing here commands steering, throttle, brake or gear. DAS_control (0x2B9) is
deliberately not sent — it carries accel and jerk limits and is a control frame,
handled as such even in the original Tinkla implementation.

Frames and rates, off a 100-tick cycle at the controller's update rate:
  DAS_lanes, DAS_object, DAS_telemetry   10 Hz
  DAS_status, DAS_status2                10 Hz
  DAS_bodyControls                        1 Hz (ticks 20, 70) + disengage edge
  warningMatrix0 / 1 / 3                  1 Hz (ticks 10, 20, 30)
  NAP Buddy status frame (0x659)         10 Hz

Every frame needs a matching entry in PREAP_TX_MSGS in
opendbc/safety/modes/tesla_preap.h, or the panda drops it and nothing renders.

Derived from the Tinkla Buddy IC work (BogGyver/openpilot tesla_unity_dev) via
MagZu/opendbc#1. The encodings and tick schedule are kept as proven; the data
sources are rewired to CarControl / CarControlSP so opendbc stays standalone.
"""
from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.preap.nap_conf import nap_conf

# DAS_status2.DAS_accSpeedLimit is a 10-bit mph signal at 0.2 scale. The packer
# takes physical units, so SNA (raw 1023) is 1023 * 0.2 mph.
DAS_ACC_SPEED_SNA = 204.6
# openpilot's "no set speed" sentinel, V_CRUISE_UNSET in selfdrive/car/cruise.py.
V_CRUISE_UNSET_KPH = 250.0

VisualAlert = structs.CarControl.HUDControl.VisualAlert

# Pre-AP renders on chassis bus 0.
CHASSIS_BUS = 0

# The IC draws the path at 2x scale, so the polynomial is scaled to match.
IC_LANE_SCALE = 0.5

# Lane-line probability above which the cluster is told a line exists.
LANE_LINE_PROB = 0.45
LANE_QUALITY_PROB = 0.25

# How often to re-read the toggle, in ticks of a 100Hz control loop. Reading the
# params filesystem every tick costs enough to trip "system lagging" on a comma 3.
TOGGLE_POLL_TICKS = 50

# Cluster warning latch, in ticks. A warning stays lit this long after it clears
# so a single-frame blip is still readable.
WARNING_LATCH_TICKS = 200


def _clip(v, lo, hi):
  return lo if v < lo else (hi if v > hi else v)


class NapBuddyHUD:
  """Builds the NAP Buddy IC frame set. One instance per CarController."""

  def __init__(self, CP, tesla_can):
    self.CP = CP
    self.tesla_can = tesla_can

    self.tick = 0
    self.warning_ticks = 0
    self.prev_steering = False

    # Toggle is polled, not read every tick. Start False so nothing is emitted
    # before the first read.
    self._enabled_cached = False
    self._debug = {}
    self._pedal_cached = False
    self._toggle_ticks = 0

    # DAS_lanes carries its own rolling counter. Tinkla had the panda firmware
    # increment it; we do not run that firmware, so a fixed counter makes every
    # lane frame byte-identical and the cluster stops redrawing the path after
    # the first one.
    self.lanes_idx = 0

    # Lane state, held between updates so the cluster keeps the last good path
    # if a model frame is missed.
    self.lane_width = 4.0
    self.left_line = 0
    self.right_line = 0
    self.left_quality = 0
    self.right_quality = 0
    self.curv = [0.0, 0.0, 0.0, 0.0]

    # Speed limit for the cluster's road-sign widget, km/h. 0 means no sign.
    self.speed_limit_kph = 0

  def _refresh_toggle(self):
    """Re-read the toggle periodically, not every tick.

    CarController.update runs at 100Hz, and nap_conf reads go to the params
    filesystem. Polling that 100 times a second is enough to blow the control
    loop's realtime budget on a comma 3 and raise "system lagging". Every
    TOGGLE_POLL_TICKS is twice a second, which is plenty for a settings toggle.
    """
    if self._toggle_ticks <= 0:
      self._toggle_ticks = TOGGLE_POLL_TICKS
      self._enabled_cached = nap_conf.buddy_ic_integration
      # Polled here too rather than read per-frame; it only feeds a display bit.
      self._pedal_cached = nap_conf.use_pedal
      self._debug = nap_conf.buddy_ic_debug_overrides
    self._toggle_ticks -= 1
    return self._enabled_cached

  @property
  def enabled(self):
    return self._enabled_cached

  def _update_lanes(self, CC_SP):
    """Take lane geometry from CarControlSP, if the openpilot side supplies it.

    The cubic path coefficients are computed where modelV2 lives and passed
    through, rather than opening a SubMaster inside opendbc. Absent the payload
    the cluster gets a straight, flat path — lanes simply do not curve.
    """
    lanes = getattr(CC_SP, "napBuddyLanes", None)
    if lanes is None or not getattr(lanes, "valid", False):
      # No usable fit this tick. Keep the last good path rather than snapping
      # the drawn lane to zero on a single dropped model frame.
      return

    self.lane_width = float(lanes.laneWidth) or 4.0
    self.left_line = 1 if float(lanes.leftLaneProb) > LANE_LINE_PROB else 0
    self.right_line = 1 if float(lanes.rightLaneProb) > LANE_LINE_PROB else 0
    self.left_quality = 1 if float(lanes.leftEdgeProb) > LANE_QUALITY_PROB else 0
    self.right_quality = 1 if float(lanes.rightEdgeProb) > LANE_QUALITY_PROB else 0

    # Bounds match what the DAS_lanes signals can represent.
    self.curv = [
      _clip(float(lanes.c0), -3.5, 3.5),
      _clip(float(lanes.c1), -0.2, 0.2),
      _clip(float(lanes.c2), -0.0025, 0.0025),
      _clip(float(lanes.c3), -0.00003, 0.00003),
    ]

  def _update_speed_limit(self, CC_SP):
    """Speed limit for the cluster road-sign widget.

    Sourced from sunnypilot's speed-limit pipeline in the openpilot layer. 0 when
    there is no limit available, which the cluster renders as no sign.
    """
    limit = getattr(CC_SP, "napBuddySpeedLimit", 0.0)
    try:
      limit_kph = int(round(float(limit) * CV.MS_TO_KPH))
    except (TypeError, ValueError):
      limit_kph = 0
    self.speed_limit_kph = limit_kph if 0 < limit_kph <= 160 else 0

  def _lead_frame(self, CC_SP):
    """DAS_object (0x309) — lead car marker.

    leadOne/leadTwo already reach opendbc on CarControlSP, so no extra plumbing
    is needed. Offsets are relative to the path centre so the marker sits on the
    drawn path rather than beside it.
    """
    lead1 = getattr(CC_SP, "leadOne", None)
    lead2 = getattr(CC_SP, "leadTwo", None)
    c0 = self.curv[0]

    def unpack(lead, lead_id):
      if lead is None or not getattr(lead, "status", False):
        return 0, 0, 0.0, 0.0, 0
      return (
        2,                                                   # vehicle class
        lead_id,
        float(_clip(lead.dRel, 0, 126)),
        float(_clip(c0 - lead.yRel, -22.05, 22.4)),
        int(_clip(int(lead.vRel), -30, 26)),
      )

    c1, id1, dx1, dy1, vx1 = unpack(lead1, 1)
    c2, id2, dx2, dy2, vx2 = unpack(lead2, 2)

    return self.tesla_can.create_lead_car_object_message(
      0,  # m0: lead-vehicle multiplexer
      c1, id1, 0, dx1, vx1, dy1,
      c2, id2, 0, dx2, vx2, dy2,
      CHASSIS_BUS,
    )

  def _status_frames(self, CC, CC_SP, CS, messages):
    """DAS_status / DAS_status2 — the AP status area of the cluster."""
    hud = CC.hudControl

    # The cluster's wheel tracks *steering*, not full engagement. With MADS the
    # car can be steering with cruise off (autosteer-only), where CC.enabled is
    # False -- keying off it drops the wheel whenever cruise is not engaged.
    steering = bool(CC.latActive)

    # DAS_op_status, per the Tesla encoding:
    #   0 disabled  1 unavailable  2 available  3 active nominal
    #   4 active restricted  5 active nav  8 aborting  9 aborted  14 fault
    # The cluster draws the steering wheel grey on 2 (available) and lights it
    # blue on 5. 1 (unavailable) draws nothing.
    engageable = bool(getattr(CC_SP, "napBuddyEngageable", False))
    if steering:
      op_status = 5
    elif engageable:
      op_status = 2
    else:
      op_status = 1
    csa_state = 2 if steering else (1 if engageable else 0)

    op_status = self._debug.get("op_status", op_status)
    csa_state = self._debug.get("csa_state", csa_state)

    collision_warning = 1 if hud.visualAlert == VisualAlert.fcw else 0

    # DAS_hands_on_state: 2 is the normal "hands detected" state. 3 flashes the
    # lamp at the top of the cluster when the driver is overriding or openpilot
    # is asking for hands. 0 is not a valid resting value.
    # 2 means "hands required, not detected". Reporting that while openpilot is
    # not steering contradicts op_status and may suppress the cluster's own
    # availability drawing, so report "not required" until it actually steers.
    hands_on_state = 2 if steering else 0
    if hud.visualAlert == VisualAlert.steerRequired:
      hands_on_state = 3
    elif steering and CS.out.steeringPressed:
      hands_on_state = 3

    # Set speed readout. DAS_accSpeedLimit is mph at 0.2 scale -- feeding it kph
    # overstates the number by 1.6x. The DBC calls raw 0 "NONE", but the cluster
    # renders that as a bogus max readout instead of hiding the field, so send
    # the documented SNA whenever there is no set speed to show.
    set_speed_kph = 0.0
    if CS.out.cruiseState.enabled:
      v_kph = max(0.0, float(hud.setSpeed) * CV.MS_TO_KPH)
      # openpilot parks hudControl.setSpeed at V_CRUISE_UNSET (255 kph) when no
      # speed is set. Converted to mph that is 158.4, which looks like a real
      # value -- it is what put "255" on the cluster. Only show a genuine speed,
      # which also keeps the readout off under autosteer-only, where the pre-AP
      # spoofer reports cruiseState.enabled with no set speed.
      if 0.0 < v_kph < V_CRUISE_UNSET_KPH:
        set_speed_kph = v_kph
    set_speed = set_speed_kph * CV.KPH_TO_MPH if set_speed_kph > 0.0 else DAS_ACC_SPEED_SNA

    # DAS_alca_state, from lane availability:
    #   1 unavailable (no lanes)  6 left only  7 right only  8 both
    if self.left_quality and self.right_quality:
      alca_state = 8
    elif self.left_quality:
      alca_state = 6
    elif self.right_quality:
      alca_state = 7
    else:
      alca_state = 1

    ldw_status = 1 if (hud.leftLaneDepart or hud.rightLaneDepart) else 0

    messages.append(self.tesla_can.create_das_status(
      op_status,          # DAS_op_status
      collision_warning,  # DAS_collision_warning
      ldw_status,         # DAS_ldwStatus
      hands_on_state,     # DAS_hands_on_state
      alca_state,         # DAS_alca_state
      0,                  # blindSpotLeft  — no BSM source on Pre-AP
      0,                  # blindSpotRight
      self.speed_limit_kph,
      1 if self.speed_limit_kph > 0 else 0,  # DAS_fleetSpeedState
      CHASSIS_BUS,
    ))
    messages.append(self.tesla_can.create_das_status2(
      csa_state, set_speed, collision_warning, CHASSIS_BUS,
    ))

    # Handed back so the 0x659 bridge frame renders from the same state rather
    # than recomputing its own, which is how it ended up disagreeing with the
    # cluster frames about both availability and the set speed.
    return {
      "op_status": op_status,
      "csa_state": csa_state,
      "hands_on_state": hands_on_state,
      "alca_state": alca_state,
      "set_speed_kph": set_speed_kph,
      "engageable": engageable,
      "steering": steering,
    }

  def update(self, CC, CC_SP, CS):
    """Build this tick's IC frames.

    Returns a list of (addr, data, bus) tuples for can_sends. Returns empty when
    the toggle is off, so the feature costs nothing until enabled.
    """
    self.tick = (self.tick + 1) % 100
    if self.warning_ticks > 0:
      self.warning_ticks -= 1
    self._refresh_toggle()

    messages = []
    if not self.enabled:
      self.prev_steering = bool(CC.latActive)
      return messages

    # Same signal the cluster wheel uses -- see _status_frames.
    steering = bool(CC.latActive)
    hud = CC.hudControl
    disengage_edge = self.prev_steering and not steering

    self._update_lanes(CC_SP)
    self._update_speed_limit(CC_SP)

    # Warning bits the cluster can show. Kept minimal and derived from state
    # openpilot already exposes; the rest of the Tesla warning matrix is left at
    # zero rather than inventing values for it.
    lane_depart = 1 if (hud.leftLaneDepart or hud.rightLaneDepart) else 0
    if lane_depart and self.warning_ticks == 0:
      self.warning_ticks = WARNING_LATCH_TICKS
    warning_active = self.warning_ticks > 0

    # --- 10 Hz block -------------------------------------------------------
    if self.tick % 10 == 0:
      messages.append(self.tesla_can.create_lane_message(
        self.lane_width, self.right_line, self.left_line, 50,
        self.curv[0], self.curv[1], self.curv[2], self.curv[3],
        self.left_quality, self.right_quality, CHASSIS_BUS, self.lanes_idx,
      ))
      self.lanes_idx = (self.lanes_idx + 1) % 16
      messages.append(self.tesla_can.create_telemetry_road_info(
        self.left_line, self.right_line,
        self.left_quality, self.right_quality, 0, CHASSIS_BUS,
      ))
      messages.append(self._lead_frame(CC_SP))
      st = self._status_frames(CC, CC_SP, CS, messages)

      # NAP Buddy status frame. This is what the bridge renders from, so it has
      # to agree with the cluster frames -- it previously recomputed its own
      # state and disagreed on both counts. apUnavailable/autopilot_disabled
      # were keyed to "is steering", so they claimed AP was unavailable whenever
      # openpilot was merely idle-but-engageable, which is what kept the grey
      # wheel from ever appearing. And the speed bytes took the raw setSpeed,
      # so V_CRUISE_UNSET went out as a literal 255 in byte 1.
      ap_available = st["steering"] or st["engageable"]

      # The ACC widget must not be drawn under autosteer-only: there is no set
      # speed then, and the bridge renders byte 1 regardless -- which is what
      # showed "255 max" while the V_CRUISE_UNSET sentinel leaked, and "0 max"
      # once it was filtered. Every field that asserts cruise is doing something
      # is keyed to having a real set speed, rather than to steering.
      cruise_active = st["set_speed_kph"] > 0.0
      speed_control_enabled = 1 if cruise_active else 0
      adaptive_cruise = 1 if cruise_active else 0
      cc_state = 2 if cruise_active else (1 if st["engageable"] else 0)
      acc_speed = st["set_speed_kph"]

      speed_control_enabled = self._debug.get("speed_control_enabled", speed_control_enabled)
      adaptive_cruise = self._debug.get("adaptive_cruise", adaptive_cruise)
      cc_state = self._debug.get("cc_state", cc_state)
      acc_speed = self._debug.get("acc_speed", acc_speed)
      pcc_available = self._debug.get("pcc_available", 1)

      messages.append(self.tesla_can.create_fake_DAS_msg(
        speed_control_enabled,
        0,                            # speed override
        0 if ap_available else 1,     # AP unavailable
        1 if hud.visualAlert == VisualAlert.fcw else 0,
        st["op_status"],
        acc_speed,
        0,                            # turn signal needed
        1 if hud.visualAlert == VisualAlert.fcw else 0,
        adaptive_cruise,
        st["hands_on_state"],
        cc_state,
        pcc_available,
        st["alca_state"],
        acc_speed,
        0,                            # legal speed limit: no map source
        0.0,                          # apply angle: steering is not driven from here
        0,                            # enable steer control: likewise
        1 if self._pedal_cached else 0,
        0 if ap_available else 1,     # autopilot disabled
        CHASSIS_BUS,
      ))

    # --- 1 Hz block --------------------------------------------------------
    # DAS_bodyControls (0x3E9) is deliberately NOT sent here. PreAPCarController
    # already transmits it every 10 frames to drive the turn indicator, and a
    # second sender on the same arb-ID would break its counter sequence. The
    # cluster gets that frame from there.

    if self.tick == 10 or disengage_edge:
      messages.append(self.tesla_can.create_das_warningMatrix0(
        0, 0, 0, CHASSIS_BUS,
      ))
    if self.tick == 20 or disengage_edge:
      messages.append(self.tesla_can.create_das_warningMatrix1(CHASSIS_BUS))
    if self.tick == 30 or disengage_edge:
      messages.append(self.tesla_can.create_das_warningMatrix3(
        0, 0, 0, 0,
        1 if (warning_active and lane_depart) else 0,  # LKAS unavailable / lane depart
        0, 0, 0, 0, 0, 0, 0, 0, CHASSIS_BUS,
      ))

    self.prev_steering = steering
    return messages
