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

VisualAlert = structs.CarControl.HUDControl.VisualAlert

# Pre-AP renders on chassis bus 0.
CHASSIS_BUS = 0

# The IC draws the path at 2x scale, so the polynomial is scaled to match.
IC_LANE_SCALE = 0.5

# Lane-line probability above which the cluster is told a line exists.
LANE_LINE_PROB = 0.45
LANE_QUALITY_PROB = 0.25

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
    self.prev_enabled = False

    # Lane state, held between updates so the cluster keeps the last good path
    # if a model frame is missed.
    self.lane_width = 4.0
    self.left_line = 0
    self.right_line = 0
    self.left_quality = 0
    self.right_quality = 0
    self.curv = [0.0, 0.0, 0.0, 0.0]

  @property
  def enabled(self):
    """Toggle state. Read every update so it takes effect without a restart."""
    return nap_conf.buddy_ic_integration

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

  def _status_frames(self, CC, CS, messages):
    """DAS_status / DAS_status2 — the AP status area of the cluster."""
    hud = CC.hudControl
    enabled = CC.enabled

    # op_status: 2 = available but off, 5 = actively steering.
    op_status = 5 if enabled else 2
    collision_warning = 1 if hud.visualAlert == VisualAlert.fcw else 0

    # Cruise set speed for the cluster readout. hudControl carries it in m/s.
    set_speed = max(0.0, float(hud.setSpeed) * CV.MS_TO_KPH)
    if set_speed > 250:  # SNA / not set
      set_speed = 0.0

    # alca_state 1 = unavailable, no lane change offered. Lane-change rendering
    # is not part of this port.
    alca_state = 1

    # ldw_status: lane departure warning, straight from hudControl.
    ldw_status = 1 if (hud.leftLaneDepart or hud.rightLaneDepart) else 0

    messages.append(self.tesla_can.create_das_status(
      op_status,          # DAS_op_status
      collision_warning,  # DAS_collision_warning
      ldw_status,         # DAS_ldwStatus
      0,                  # DAS_hands_on_state — not modelled here
      alca_state,         # DAS_alca_state
      0,                  # blindSpotLeft  — no BSM source on Pre-AP
      0,                  # blindSpotRight
      0,                  # DAS_speed_limit_kph — no map speed-limit source
      0,                  # DAS_fleetSpeedState
      CHASSIS_BUS,
    ))
    messages.append(self.tesla_can.create_das_status2(
      0, set_speed, collision_warning, CHASSIS_BUS,
    ))

  def update(self, CC, CC_SP, CS):
    """Build this tick's IC frames.

    Returns a list of (addr, data, bus) tuples for can_sends. Returns empty when
    the toggle is off, so the feature costs nothing until enabled.
    """
    self.tick = (self.tick + 1) % 100
    if self.warning_ticks > 0:
      self.warning_ticks -= 1

    messages = []
    if not self.enabled:
      self.prev_enabled = CC.enabled
      return messages

    enabled = CC.enabled
    hud = CC.hudControl
    disengage_edge = self.prev_enabled and not enabled

    self._update_lanes(CC_SP)

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
        self.left_quality, self.right_quality, CHASSIS_BUS, 1,
      ))
      messages.append(self.tesla_can.create_telemetry_road_info(
        self.left_line, self.right_line,
        self.left_quality, self.right_quality, 0, CHASSIS_BUS,
      ))
      messages.append(self._lead_frame(CC_SP))
      self._status_frames(CC, CS, messages)

      # NAP Buddy status frame. Carries display state for the bridge.
      messages.append(self.tesla_can.create_fake_DAS_msg(
        1 if enabled else 0,   # speed control enabled
        0,                     # speed override
        0 if enabled else 1,   # AP unavailable
        1 if hud.visualAlert == VisualAlert.fcw else 0,
        5 if enabled else 2,   # op status
        max(0.0, float(hud.setSpeed) * CV.MS_TO_KPH),
        0,                     # turn signal needed
        1 if hud.visualAlert == VisualAlert.fcw else 0,
        1,                     # adaptive cruise available
        0,                     # hands on state
        2 if enabled else 0,   # cc state
        1,                     # pedal available
        1,                     # alca state: unavailable
        max(0.0, float(hud.setSpeed) * CV.MS_TO_KPH),
        0,                     # legal speed limit: no map source
        0.0,                   # apply angle: steering is not driven from here
        0,                     # enable steer control: likewise
        1 if nap_conf.use_pedal else 0,
        0 if enabled else 1,
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

    self.prev_enabled = enabled
    return messages
