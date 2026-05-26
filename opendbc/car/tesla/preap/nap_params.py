"""
NAP (NotAutopilot) Parameter Keys

Single source of truth for all NAP param key names used by
the UI settings panel and Tesla Pre-AP car code.

Storage: openpilot Params system (params_keys.h)
"""


class NAPParamKeys:
  # Longitudinal Control
  ADAPTIVE_ACCEL = "NAPAdaptiveAccel"
  PEDAL_ENABLED = "NAPPedalEnabled"
  FOLLOW_DISTANCE = "NAPFollowDistance"
  # Pedal Hardware
  PEDAL_PROFILE = "NAPPedalProfile"
  PEDAL_CAN_BUS = "NAPPedalCanBus"
  PEDAL_CALIB_DONE = "NAPPedalCalibDone"
  PEDAL_CALIB_MIN = "NAPPedalCalibMin"
  PEDAL_CALIB_MAX = "NAPPedalCalibMax"
  PEDAL_CALIB_FACTOR = "NAPPedalCalibFactor"
  PEDAL_CALIB_ZERO = "NAPPedalCalibZero"

  # Radar
  RADAR_ENABLED = "NAPRadarEnabled"
  RADAR_BEHIND_NOSECONE = "NAPRadarBehindNosecone"
  RADAR_OFFSET = "NAPRadarOffset"

  # iBooster / Braking
  IBOOSTER_ENABLED = "NAPiBoosterEnabled"
  BRAKE_FACTOR = "NAPBrakeFactor"

  # Advanced
  FORCE_PRE_AP = "NAPForcePreAP"

  # Tinkla Buddy IC integration — IC-rendering via DAS-frames on chassis bus 0.
  # Default off. Display-only, risk-tier 3 (Buddy IC does not affect engage or safety).
  TINKLA_IC_INTEGRATION = "NAPTinklaICIntegration"

  # Tesla IC native road-sign widget fallback (kph). Used when Tesla DI's
  # UI_gpsVehicleSpeed.UI_mppSpeedLimit reports 0 (no GPS-fix, no nav-DB hit).
  # Default 0 = no sign shown when no GPS data. Risk-tier 3 (display-only).
  ROAD_SIGN_FALLBACK_KPH = "NAPRoadSignFallbackKph"


# Default values matching params_keys.h declarations
DEFAULTS = {
  NAPParamKeys.ADAPTIVE_ACCEL: True,
  NAPParamKeys.PEDAL_ENABLED: False,
  NAPParamKeys.FOLLOW_DISTANCE: 4,
  NAPParamKeys.PEDAL_PROFILE: 4,
  NAPParamKeys.PEDAL_CAN_BUS: 2,
  NAPParamKeys.PEDAL_CALIB_DONE: False,
  NAPParamKeys.PEDAL_CALIB_MIN: -3.0,
  NAPParamKeys.PEDAL_CALIB_MAX: 99.6,
  NAPParamKeys.PEDAL_CALIB_FACTOR: 1.0,
  NAPParamKeys.PEDAL_CALIB_ZERO: 0.0,
  NAPParamKeys.RADAR_ENABLED: False,
  NAPParamKeys.RADAR_BEHIND_NOSECONE: False,
  NAPParamKeys.RADAR_OFFSET: 0.0,
  NAPParamKeys.IBOOSTER_ENABLED: False,
  NAPParamKeys.BRAKE_FACTOR: 1.0,
  NAPParamKeys.FORCE_PRE_AP: False,
  NAPParamKeys.TINKLA_IC_INTEGRATION: False,  # Buddy IC-rendering toggle, default off
  NAPParamKeys.ROAD_SIGN_FALLBACK_KPH: 0,  # 0 = no sign when GPS SNA
}
