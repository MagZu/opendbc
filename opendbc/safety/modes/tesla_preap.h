#pragma once

// Pre-AP safety code — GTW emulation, stalk logic, gear/door checks,
// pedal interceptor, and engagement state.
//
// Included by tesla_legacy.h AFTER utility functions are defined.
// This file should NOT be modified when upstream updates tesla_legacy.h —
// all Pre-AP logic is self-contained here.

// ============================================
// Pre-AP State Variables
// ============================================

static int pedal_can = -1;
static int pedal_pressed = 0;

static int radar_epas_type = 0;
static int radar_position = 0;

// Safety state: gear, doors
static int tesla_gear = 4;       // Initialize to Drive (4) to avoid false disables on startup
static int tesla_gear_prev = 4;  // Track previous gear for edge detection
static bool tesla_doors_open = false;

// Stalk echo filter: ignore cancel echoes within this window of a spoof or engage
static uint32_t tesla_last_stalk_engage_us = 0;
#define TESLA_CANCEL_ECHO_WINDOW_US 600000U  // 600ms

// Radar emulation state
static int tesla_radar_status = 0;          // 0=unknown, 1=init(0x631), 2=active(0x300)
static uint32_t tesla_last_radar_signal = 0;
#define TESLA_RADAR_TIMEOUT 10000000U       // 10 seconds in microseconds

// ============================================
// Pre-AP Init
// ============================================

static void tesla_preap_init_state(void) {
  tesla_gear = 4;
  tesla_gear_prev = 4;
  tesla_doors_open = false;
  pedal_can = -1;
  pedal_pressed = 0;
  tesla_radar_status = 0;
  tesla_last_radar_signal = 0;
  radar_position = tesla_radar_behind_nosecone ? 1 : 0;
}

// ============================================
// Pre-AP GTW Emulation (rx_all hook body)
// ============================================

static void tesla_preap_handle_forwarding(const CANPacket_t *to_fwd) {
  int bus_num = GET_BUS(to_fwd);
  int addr = GET_ADDR(to_fwd);

  if (bus_num == 0 && tesla_radar_emulation) {
    // Radar forwarding CAN0 -> CAN1 (full GTW emulation).
    // Intercept chassis messages and forward to radar bus with re-addressed CAN IDs.
    // Without this, the Bosch radar has no vehicle speed, steering angle, or brake data.

    // Group A: Simple re-addresses (copy data verbatim, change CAN ID)
    switch (addr) {
      case 0x45:   tesla_radar_readdr(to_fwd, 0x219); break;  // STW_ACTN_RQ -> stalk
      case 0x108:  tesla_radar_readdr(to_fwd, 0x109); break;  // DI_torque1
      case 0x145:  tesla_radar_readdr(to_fwd, 0x149); break;  // ESP_145h
      case 0x20A:  tesla_radar_readdr(to_fwd, 0x159); break;  // BrakeMessage -> ESP_C
      case 0x308:  tesla_radar_readdr(to_fwd, 0x209); break;  // GTW_odo
      case 0x30A:  tesla_radar_readdr(to_fwd, 0x2D9); break;  // BC_status
      case 0x405:  tesla_radar_readdr(to_fwd, 0x2B9); break;  // VIP_405HS
      default: break;
    }

    // Group B: Modified data forwards

    if (addr == 0x398) { // GTW_carConfig -> 0x2A9
      CANPacket_t to_send;
      to_send.returned = 0U;
      to_send.rejected = 0U;
      to_send.extended = to_fwd->extended;
      to_send.bus = 1;
      to_send.data_len_code = to_fwd->data_len_code;
      uint32_t RDLR = GET_BYTES_04(to_fwd);
      uint32_t RDHR = GET_BYTES_48(to_fwd);

      // Set country=US (0x100), radar_type=Bosch (0x440), radar position and EPAS type
      RDLR = (RDLR & 0xFFFFF33F) | 0x100 | 0x440;
      RDHR = (RDHR & 0xCFFF0F0F) | 0x10000000 | (radar_position << 4) | (radar_epas_type << 12);

      to_send.addr = 0x2A9;
      WORD_TO_BYTE_ARRAY(&to_send.data[4], RDHR);
      WORD_TO_BYTE_ARRAY(&to_send.data[0], RDLR);
      to_send.data[7] = tesla_legacy_compute_checksum(&to_send);
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&to_send);
      can_send(&to_send, 1, true);
#endif
    }

    if (addr == 0x0E) { // STW_ANGLHP_STAT -> 0x199
      CANPacket_t to_send;
      to_send.returned = 0U;
      to_send.rejected = 0U;
      to_send.extended = to_fwd->extended;
      to_send.bus = 1;
      to_send.data_len_code = to_fwd->data_len_code;
      uint32_t RDLR = GET_BYTES_04(to_fwd);
      uint32_t RDHR = GET_BYTES_48(to_fwd);

      to_send.addr = 0x199;
      // Check if angular speed field (bits 29:16) is SNA (0x3FFF)
      if (((RDLR >> 16) & 0xFF3F) == 0xFF3F) {
        // Replace with zero angular change
        RDLR = (RDLR & 0x00C0FFFF) | (0x0020 << 16);
        // Remove CRC and sensor ID, force DELPHI (0x04)
        RDHR = (RDHR & 0x00FFFFF0) | 0x00000004;
        // Recompute CRC8 into byte 7
        int crc = tesla_legacy_compute_crc(RDLR, RDHR, 7);
        RDHR = RDHR | ((uint32_t)crc << 24);
      }
      WORD_TO_BYTE_ARRAY(&to_send.data[4], RDHR);
      WORD_TO_BYTE_ARRAY(&to_send.data[0], RDLR);
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&to_send);
      can_send(&to_send, 1, true);
#endif
    }

    // Group C: Synthetic message generation

    if (addr == 0x115) { // ESP_115h -> 0x129 + synthetic 0x1A9
      // Simple re-address 0x115 -> 0x129
      tesla_radar_readdr(to_fwd, 0x129);

      // Build synthetic DI_espControl (0x1A9), 5 bytes
      uint32_t RDHR_src = GET_BYTES_48(to_fwd);
      int counter = ((RDHR_src & 0xF0) >> 4) & 0x0F;
      uint32_t syn_RDLR = 0x000C0000U | ((uint32_t)counter << 28);
      int cksm = (0x38 + 0x0C + (counter << 4)) & 0xFF;
      uint32_t syn_RDHR = (uint32_t)cksm;

      CANPacket_t to_send;
      to_send.returned = 0U;
      to_send.rejected = 0U;
      to_send.extended = 0;
      to_send.bus = 1;
      to_send.addr = 0x1A9;
      to_send.data_len_code = 5;  // 5 bytes -> DLC 5
      WORD_TO_BYTE_ARRAY(&to_send.data[0], syn_RDLR);
      WORD_TO_BYTE_ARRAY(&to_send.data[4], syn_RDHR);
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&to_send);
      can_send(&to_send, 1, true);
#endif
    }

    if (addr == 0x118) { // DI_torque2 -> 0x119 + synthetic 0x169
      // Simple re-address 0x118 -> 0x119
      tesla_radar_readdr(to_fwd, 0x119);

      // Build synthetic ESP_wheelSpeeds (0x169), 8 bytes
      uint32_t RDLR = GET_BYTES_04(to_fwd);
      int ws_counter = GET_BYTES_48(to_fwd) & 0x0F;

      // Extract DI_vehicleSpeed from bits [27:16]
      int raw_speed = (int)((0xFFF0000U & RDLR) >> 16);

      int speed;
      if (raw_speed == 0xFFF) {
        speed = 0x1FFF;  // SNA
      } else {
        // Convert MPH -> KPH using integer math, then encode (÷ 0.04)
        // raw * 0.05 - 25 = MPH; * 1.609 = KPH; / 0.04 encodes
        int mph_x100 = raw_speed * 5 - 2500;
        int kph_x100 = mph_x100 * 1609 / 1000;
        if (kph_x100 < 0) {
          kph_x100 = 0;
        }
        speed = (kph_x100 / 4) & 0x1FFF;
      }

      // Pack 4 identical 13-bit wheel speeds
      uint32_t ws_RDLR = (uint32_t)(speed | (speed << 13) | (speed << 26));
      uint32_t ws_RDHR = (uint32_t)((speed >> 6) | (speed << 7) | (ws_counter << 20)) & 0x00FFFFFFU;

      // Checksum: base 0x76, sum bytes 0-6, place in byte 7
      int ws_cksm = 0x76;
      ws_cksm = (ws_cksm + (int)(ws_RDLR & 0xFF) + (int)((ws_RDLR >> 8) & 0xFF) + (int)((ws_RDLR >> 16) & 0xFF) + (int)((ws_RDLR >> 24) & 0xFF)) & 0xFF;
      ws_cksm = (ws_cksm + (int)(ws_RDHR & 0xFF) + (int)((ws_RDHR >> 8) & 0xFF) + (int)((ws_RDHR >> 16) & 0xFF)) & 0xFF;
      ws_RDHR = ws_RDHR | ((uint32_t)ws_cksm << 24);

      CANPacket_t to_send;
      to_send.returned = 0U;
      to_send.rejected = 0U;
      to_send.extended = 0;
      to_send.bus = 1;
      to_send.addr = 0x169;
      to_send.data_len_code = 8;  // 8 bytes -> DLC 8
      WORD_TO_BYTE_ARRAY(&to_send.data[0], ws_RDLR);
      WORD_TO_BYTE_ARRAY(&to_send.data[4], ws_RDHR);
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&to_send);
      can_send(&to_send, 1, true);
#endif
    }
  }

  // Radar status tracking (CAN1 -> informational only)
  if (bus_num == 1 && tesla_radar_emulation) {
    if (addr == 0x631 && tesla_radar_status == 0) {
      tesla_radar_status = 1;  // init
      tesla_last_radar_signal = microsecond_timer_get();
    }
    if (addr == 0x300 && tesla_radar_status == 1) {
      tesla_radar_status = 2;  // active
      tesla_last_radar_signal = microsecond_timer_get();
    }
  }
}

// ============================================
// Pre-AP RX Hook
// ============================================

static void tesla_preap_rx_hook(const CANPacket_t *msg) {
  // EPAS (0x370): steering angle, hands-on level, disengage detection
  if ((msg->bus == 0U) && (msg->addr == 0x370U)) {
    const int angle_meas_new = (((msg->data[4] & 0x3FU) << 8) | msg->data[5]) - 8192U;
    update_sample(&angle_meas, angle_meas_new);

    const int hands_on_level = msg->data[4] >> 6;
    const int eac_status = msg->data[6] >> 5;
    const int eac_error_code = msg->data[2] >> 4;

    steering_disengage = (hands_on_level >= 3) || ((eac_status == 0) && (eac_error_code == 9));

    // Pre-AP re-arm fix:
    // Steering disengage drops controls_allowed in generic_rx_checks, but Pre-AP uses
    // stalk edges (pcm_cruise_check) to re-enable controls. If cruise_engaged_prev is
    // still true, the next stalk pull(true) is not a rising edge and controls stay off.
    // Force a local "cruise disengaged" on steering-disengage rising edge so the next
    // stalk pull can reliably re-arm controls_allowed.
    if (steering_disengage && !steering_disengage_prev) {
      pcm_cruise_check(false);
    }
  }

  // Vehicle speed (ESP_B: 0x155)
  if ((msg->bus == 0U) && (msg->addr == 0x155U)) {
    float speed = ((msg->data[6] | (msg->data[5] << 8)) * 0.01) * KPH_TO_MS;
    UPDATE_VEHICLE_SPEED(speed);
  }

  // Gas pressed from DI_torque1 (0x108) — only when pedal interceptor is not active
  if ((msg->bus == 0U) && (msg->addr == 0x108U)) {
    if (!tesla_enable_pedal) {
      gas_pressed = msg->data[6] != 0U;
    }
  }

  // Pedal Interceptor (0x552)
  if (tesla_enable_pedal && (msg->addr == 0x552)) {
    int pedal_val = ((msg->data[0] << 8) | msg->data[1]);
    pedal_pressed = pedal_val;
    gas_pressed = (pedal_pressed > 450);
    if (pedal_can == -1) {
      pedal_can = msg->bus;
    }
  }

  // Brake (0x20a) — force false for ALL Pre-AP modes (pedal and non-pedal).
  // Selfdrive drops longitudinal only on brake; lateral stays active.
  // Without this, generic_rx_checks() drops controls_allowed every frame
  // while vehicle_moving && brake_pressed, causing controlsMismatch.
  if ((msg->bus == 0U) && (msg->addr == 0x20aU)) {
    brake_pressed = false;
  }

  // Cruise state (DI_state: 0x368) — track vehicle_moving but DON'T call
  // pcm_cruise_check (Pre-AP uses stalk edges instead)
  if ((msg->bus == 0U) && (msg->addr == 0x368U)) {
    int cruise_state = (msg->data[1] >> 4) & 0x07U;
    vehicle_moving = cruise_state != 3;  // STANDSTILL
  }

  // Gear check (DI_torque2: 0x118) — disable on falling edge out of Drive
  if ((msg->bus == 0U) && (msg->addr == 0x118U)) {
    tesla_gear = (msg->data[1] >> 4) & 0x07;
    if ((tesla_gear_prev == 4) && (tesla_gear != 4)) {
      controls_allowed = 0;
    }
    tesla_gear_prev = tesla_gear;
  }

  // Door check (GTW_carState: 0x318)
  if ((msg->bus == 0U) && (msg->addr == 0x318U)) {
    int door_FL = (msg->data[1] >> 4) & 0x03;
    int door_FR = (msg->data[1] >> 6) & 0x03;
    int door_RL = (msg->data[2] >> 6) & 0x03;
    int door_RR = (msg->data[3] >> 5) & 0x03;
    int door_front_trunk = (msg->data[6] >> 2) & 0x03;
    int door_trunk = (msg->data[5] >> 6) & 0x03;
    tesla_doors_open = (door_FL == 1) || (door_FR == 1) || (door_RL == 1) || (door_RR == 1) || (door_front_trunk == 1) || (door_trunk == 1);
    if (tesla_doors_open) {
      controls_allowed = 0;
    }
  }

  // Stalk logic (STW_ACTN_RQ: 0x45)
  if ((msg->bus == 0U) && (msg->addr == 0x45U)) {
    int ap_lever_position = msg->data[0] & 0x3FU;
    if (ap_lever_position == 2) { // RWD = Pull toward driver = Enable
      if ((tesla_gear == 4) && !tesla_doors_open) {
        pcm_cruise_check(true);
        tesla_last_stalk_engage_us = microsecond_timer_get();
      }
    } else if (ap_lever_position == 1) { // FWD = Push away = Cancel
      // Only honor cancel outside the echo window. The CC spoof logic sends
      // fake cancel messages that echo back within ~300ms. Real driver cancels
      // happen well outside this window.
      uint32_t elapsed = microsecond_timer_get() - tesla_last_stalk_engage_us;
      if (elapsed > TESLA_CANCEL_ECHO_WINDOW_US) {
        pcm_cruise_check(false);
      }
    }
  }
}

// ============================================
// Pre-AP Forwarding Hook
// ============================================

static bool tesla_preap_fwd_hook(void) {
  // Pre-AP has a single panda with nothing on bus 2.  Returning true blocks
  // automatic bus 0→2 forwarding, which would flood a dead TX queue (~1300
  // overflows/s) and starve bus-0 TX interrupts.
  return true;
}

// ============================================
// Pre-AP TX Whitelist and RX Checks
// ============================================

// NOTE: Pre-AP Teslas do NOT have a harness relay!
// Setting check_relay=false for all Pre-AP messages to prevent false "relay malfunction" errors.
// Tinkla's code confirms: "PreAP has no relay"
static const CanMsg TESLA_TX_PREAP_MSGS[] = {
  // Core control messages (check_relay=false because no relay in Pre-AP)
  {0x488, 0, 4, .check_relay = false, .disable_static_blocking = true},  // DAS_steeringControl
  {0x2B9, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_control
  {0x214, 0, 3, .check_relay = false, .disable_static_blocking = true},  // EPB_epasControl (EPAS handshake)

  // Pedal Interceptor (both buses for compatibility)
  {0x551, 0, 6, .check_relay = false, .disable_static_blocking = true},  // Pedal on Bus 0
  {0x551, 2, 6, .check_relay = false, .disable_static_blocking = true},  // Pedal on Bus 2 (DEFAULT!)

  // Fake stalk cancel - CRITICAL: check_relay MUST be false!
  // The car constantly sends 0x45 (stalk position), so check_relay=true would
  // trigger "relay malfunction" when we try to send our fake cancel
  {0x45, 0, 8, .check_relay = false, .disable_static_blocking = true},   // STW_ACTN_RQ (fake stalk cancel)

  // IC Integration / Communication with panda (internal message)
  {0x659, 0, 8, .check_relay = false, .disable_static_blocking = true},  // Fake DAS message for pedal state
};

static RxCheck tesla_preap_rx_checks[] = {
  {.msg = {{0x370, 0, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // EPAS_sysStatus (25Hz)
  {.msg = {{0x108, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},  // DI_torque1 (100Hz)
  {.msg = {{0x118, 0, 6, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},  // DI_torque2 (100Hz)
  {.msg = {{0x20a, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // BrakeMessage (50Hz)
  {.msg = {{0x368, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DI_state (10Hz)
  {.msg = {{0x318, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // GTW_carState (10Hz)
  {.msg = {{0x45, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},    // STW_ACTN_RQ - Stalk (10Hz)
};
