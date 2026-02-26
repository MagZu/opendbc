#pragma once

#include "opendbc/safety/declarations.h"

// Legacy macros for Tinkla porting
#define GET_BYTES_04(msg) ((msg)->data[0] | ((msg)->data[1] << 8) | ((msg)->data[2] << 16) | ((msg)->data[3] << 24))
#define GET_BYTES_48(msg) ((msg)->data[4] | ((msg)->data[5] << 8) | ((msg)->data[6] << 16) | ((msg)->data[7] << 24))
#define WORD_TO_BYTE_ARRAY(dst8, src32) 0[dst8] = ((src32) & 0xFFU); 1[dst8] = (((src32) >> 8U) & 0xFFU); 2[dst8] = (((src32) >> 16U) & 0xFFU); 3[dst8] = (((src32) >> 24U) & 0xFFU)

// Forward declaration
#if defined(STM32H7) || defined(STM32F4)
void can_send(CANPacket_t *to_push, uint8_t bus_number, bool skip_tx_hook);
#endif

static bool tesla_external_panda = false;
static bool tesla_hw1 = false;
static bool tesla_hw2 = false;
static bool tesla_hw3 = false;
static bool tesla_preap = false;
static bool tesla_enable_pedal = false;
static bool tesla_radar_behind_nosecone = false;
static bool tesla_radar_emulation = false;

static int chassis_bus = 0U;
static int das_control_msg = 0x2bfU;
static int di_torque1_msg = 0x106U;

static bool tesla_legacy_stock_aeb = false;

static bool tesla_legacy_stock_lkas = false;
static bool tesla_legacy_stock_lkas_prev = false;

// Pre-AP specific state
static int pedal_can = -1;
static int pedal_pressed = 0; 

static int radar_epas_type = 0; 
static int radar_position = 0; 

// Pre-AP Safety State
static int tesla_gear = 4;  // Initialize to Drive (4) to avoid false disables on startup
static int tesla_gear_prev = 4;  // Track previous gear for edge detection
static bool tesla_doors_open = false;

// Radar emulation state
static int tesla_radar_status = 0;          // 0=unknown, 1=init(0x631), 2=active(0x300)
static uint32_t tesla_last_radar_signal = 0;
#define TESLA_RADAR_TIMEOUT 10000000U       // 10 seconds in microseconds

static uint8_t tesla_legacy_compute_checksum(const CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);
  int len = GET_LEN(to_push);
  uint8_t checksum = (uint8_t)(addr) + (uint8_t)((unsigned int)(addr) >> 8U);
  for (int i = 0; i < (len - 1); i++) {
    checksum += (uint8_t)GET_BYTE(to_push, i);
  }
  return checksum;
}

static uint8_t tesla_legacy_compute_crc(uint32_t MLB, uint32_t MHB, int msg_len) {
  static const int crc_lookup[256] = { 0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB, 
    0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76, 
    0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C, 
    0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1, 
    0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8, 
    0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65, 
    0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F, 
    0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2, 
    0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D, 
    0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50, 
    0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A, 
    0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7, 
    0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E, 
    0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43, 
    0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09, 
    0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4 };
  int crc = 0xFF;
  for (int x = 0; x < msg_len; x++) {
    int v = 0;
    if (x <= 3) {
      v = (MLB >> (x * 8)) & 0xFF;
    } else {
      v = (MHB >> ( (x-4) * 8)) & 0xFF;
    }
    crc = crc_lookup[crc ^ v];
  }
  crc = crc ^ 0xFF;
  return crc;
}

// Helper: copy data verbatim, re-address, send to bus 1
static void tesla_radar_readdr(const CANPacket_t *src, uint16_t new_addr) {
#if defined(STM32H7) || defined(STM32F4)
  CANPacket_t pkt;
  pkt.returned = 0U;
  pkt.rejected = 0U;
  pkt.extended = src->extended;
  pkt.bus = 1;
  pkt.addr = new_addr;
  pkt.data_len_code = src->data_len_code;
  for (int i = 0; i < GET_LEN(src); i++) {
    pkt.data[i] = src->data[i];
  }
  can_send(&pkt, 1, true);
#else
  (void)src;
  (void)new_addr;
#endif
}

// Handles manual forwarding modification since safety hooks don't allow modification
static void tesla_legacy_handle_forwarding(const CANPacket_t *to_fwd) {
  int bus_num = GET_BUS(to_fwd);
  int addr = GET_ADDR(to_fwd);

  if (bus_num == 0 && tesla_preap && tesla_radar_emulation) {
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

      RDLR = (RDLR & 0xFFFFF33F) | 0x100 | 0x440;
      RDHR = (RDHR & 0xCFFF0F0F) | 0x10000000 | (radar_position << 4) | (radar_epas_type << 12);

      to_send.addr = 0x2A9;
      WORD_TO_BYTE_ARRAY(&to_send.data[4], RDHR);
      WORD_TO_BYTE_ARRAY(&to_send.data[0], RDLR);
      to_send.data[7] = tesla_legacy_compute_checksum(&to_send);
#if defined(STM32H7) || defined(STM32F4)
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
      // Check if angular speed is SNA (0x3FFF)
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
      can_send(&to_send, 1, true);
#endif
    }
  }

  // Radar status tracking (CAN1 -> informational only)
  if (bus_num == 1 && tesla_preap && tesla_radar_emulation) {
    if (addr == 0x631 && tesla_radar_status == 0) {
      tesla_radar_status = 1;  // init
      tesla_last_radar_signal = microsecond_timer_get();
    }
    if (addr == 0x300 && tesla_radar_status == 1) {
      tesla_radar_status = 2;  // active
      tesla_last_radar_signal = microsecond_timer_get();
    }
  }

  // Simple forwarding 2 -> 0
  if (bus_num == 2) {
    // We need to decide what to block/forward.
    // Since we can't block selectively in fwd_hook (it blocks all or nothing per ID?), 
    // manual forwarding is safer if we want filtering.
    // But here we just want to pass everything relevant.
    
    bool forward = true;
    // Filter logic:
    if (!tesla_external_panda && !tesla_hw1 && (addr == 0x27dU)) forward = false;
    if (!tesla_external_panda && (addr == 0x488U) && !tesla_legacy_stock_lkas) forward = true; 
    
    if (forward) {
        CANPacket_t to_send;
        to_send.returned = 0U;
        to_send.rejected = 0U;
        to_send.extended = to_fwd->extended;
        to_send.bus = 0;
        to_send.addr = addr;
        to_send.data_len_code = to_fwd->data_len_code;
        uint32_t RDLR = GET_BYTES_04(to_fwd);
        uint32_t RDHR = GET_BYTES_48(to_fwd);
        WORD_TO_BYTE_ARRAY(&to_send.data[4], RDHR);
        WORD_TO_BYTE_ARRAY(&to_send.data[0], RDLR);
#if defined(STM32H7) || defined(STM32F4)
        can_send(&to_send, 0, true);
#endif
    }
  }
}

static void tesla_legacy_rx_hook(const CANPacket_t *msg) {
  // Handle forwarding (Manual injection)
  tesla_legacy_handle_forwarding(msg);

  // Steering angle: (0.1 * val) - 819.2 in deg.
  if (!tesla_external_panda && (msg->bus == 0U) && (msg->addr == 0x370U)) {
    // Store it 1/10 deg to match steering request
    const int angle_meas_new = (((msg->data[4] & 0x3FU) << 8) | msg->data[5]) - 8192U;
    update_sample(&angle_meas, angle_meas_new);

    const int hands_on_level = msg->data[4] >> 6;  // handsOnLevel
    const int eac_status = msg->data[6] >> 5;      // eacStatus
    const int eac_error_code = msg->data[2] >> 4;  // eacErrorCode

    // Disengage on normal user override, or if high angle rate fault from user overriding extremely quickly
    steering_disengage = (hands_on_level >= 3) || ((eac_status == 0) && (eac_error_code == 9));

    // Pre-AP re-arm fix:
    // Steering disengage drops controls_allowed in generic_rx_checks, but Pre-AP uses
    // stalk edges (pcm_cruise_check) to re-enable controls. If cruise_engaged_prev is
    // still true, the next stalk pull(true) is not a rising edge and controls stay off.
    // Force a local "cruise disengaged" on steering-disengage rising edge so the next
    // stalk pull can reliably re-arm controls_allowed.
    if (tesla_preap && steering_disengage && !steering_disengage_prev) {
      pcm_cruise_check(false);
    }
  }

  // Vehicle speed (ESP_B: ESP_vehicleSpeed)
  if ((!tesla_external_panda) && (msg->bus == chassis_bus) && (msg->addr == 0x155U)) {
    // Vehicle speed: (0.01 * val) * KPH_TO_MS
    float speed = ((msg->data[6] | (msg->data[5] << 8)) * 0.01) * KPH_TO_MS;
    UPDATE_VEHICLE_SPEED(speed);
  }

  // Gas pressed
  if ((tesla_external_panda || tesla_hw1) && (msg->bus == 0U) && (msg->addr == di_torque1_msg)) {
    gas_pressed = msg->data[6] != 0U;
  }

  // Gas pressed for Pre-AP
  if (tesla_preap && (msg->bus == 0U) && (msg->addr == 0x108U)) {
    if (!tesla_enable_pedal) {
      gas_pressed = msg->data[6] != 0U;
    }
  }
  
  // Pedal Interceptor
  if (tesla_preap && tesla_enable_pedal && (msg->addr == 0x552)) {
     int pedal_val = ((msg->data[0] << 8) | msg->data[1]);
     pedal_pressed = pedal_val;
     gas_pressed = (pedal_pressed > 450);

     if (pedal_can == -1) {
        pedal_can = msg->bus;
     }
  }

  if (((tesla_external_panda) && (msg->bus == 0U) && (msg->addr == 0x1f8U)) ||
     ((!tesla_external_panda) && (msg->bus == chassis_bus) && (msg->addr == 0x20aU))) {
    brake_pressed = (((msg->data[0] & 0x0CU) >> 2) != 1U);
  }

  // Pre-AP Brake Check
  // Keep controls_allowed active during brake for ALL Pre-AP modes (pedal and
  // non-pedal).  Selfdrive drops longitudinal only on brake; lateral stays
  // active.  Without this, generic_rx_checks() drops controls_allowed every
  // frame while vehicle_moving && brake_pressed, causing controlsMismatch when
  // engaging at low speed with foot near brake.
  if (tesla_preap && (msg->bus == 0U) && (msg->addr == 0x20aU)) {
    brake_pressed = false;
  }

  // Cruise
  if (((tesla_external_panda) && (msg->bus == 0U) && (msg->addr == 0x256U)) ||
     ((!tesla_external_panda) && (msg->bus == chassis_bus) && (msg->addr == 0x368U))) {
      // Cruise state
      int cruise_state = (msg->data[1] >> 4) & 0x07U;
      bool cruise_engaged = (cruise_state == 2) ||  // ENABLED
                            (cruise_state == 3) ||  // STANDSTILL
                            (cruise_state == 4) ||  // OVERRIDE
                            (cruise_state == 6) ||  // PRE_FAULT
                            (cruise_state == 7);    // PRE_CANCEL
      vehicle_moving = cruise_state != 3; // STANDSTILL
      if (!tesla_preap) {
        pcm_cruise_check(cruise_engaged);
      }
   }

  // Pre-AP Gear Check - Only disable on falling edge (was in D, now not in D)
  // This prevents race conditions where controls_allowed is constantly being reset
  if (tesla_preap && (msg->bus == 0U) && (msg->addr == 0x118U)) {
    tesla_gear = (msg->data[1] >> 4) & 0x07;
    // Only disable controls when shifting OUT of Drive, not continuously
    if ((tesla_gear_prev == 4) && (tesla_gear != 4)) {
      controls_allowed = 0;
    }
    tesla_gear_prev = tesla_gear;
  }

  // Pre-AP Door Check (using GTW_carState)
  if (tesla_preap && (msg->bus == 0U) && (msg->addr == 0x318U)) {
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

  // Pre-AP Stalk Logic
  if (tesla_preap && (msg->bus == 0U) && (msg->addr == 0x45U)) {
    int ap_lever_position = msg->data[0] & 0x3FU;
    if (ap_lever_position == 2) { // Pull forward (Enable)
      // Only enable if in Drive and Doors closed
      if ((tesla_gear == 4) && !tesla_doors_open) {
        pcm_cruise_check(true);
      }
    } else if (ap_lever_position == 1) { // Push back (Disable)
      pcm_cruise_check(false);
    }
  }

  if (msg->bus == 2U) {
    // DAS_control
    if ((tesla_external_panda || tesla_hw1) && msg->addr == das_control_msg) {
      // "AEB_ACTIVE"
      tesla_legacy_stock_aeb = (msg->data[2] & 0x03U) == 1U;
    }

    // DAS_steeringControl
    if (!tesla_external_panda && msg->addr == 0x488U) {
      int steering_control_type = msg->data[2] >> 6;
      bool tesla_legacy_stock_lkas_now = steering_control_type == 2;  // "LANE_KEEP_ASSIST"

      // Only consider rising edges while controls are not allowed
      if (tesla_legacy_stock_lkas_now && !tesla_legacy_stock_lkas_prev && !controls_allowed) {
        tesla_legacy_stock_lkas = true;
      }
      if (!tesla_legacy_stock_lkas_now) {
        tesla_legacy_stock_lkas = false;
      }
      tesla_legacy_stock_lkas_prev = tesla_legacy_stock_lkas_now;
    }
  }
}


static bool tesla_legacy_tx_hook(const CANPacket_t *msg) {
  const AngleSteeringLimits TESLA_STEERING_LIMITS = {
    .max_angle = 3600,  // 360 deg, EPAS faults above this
    .angle_deg_to_can = 10,
    .frequency = 50U,
  };

  // NOTE: based off TESLA_MODEL_S_HW3 to match openpilot
  const AngleSteeringParams TESLA_LEGACY_STEERING_PARAMS = {
    .slip_factor = -0.0005666493436310427,  // calc_slip_factor(VM)
    .steer_ratio = 15.,
    .wheelbase = 2.96,
  };

  const LongitudinalLimits TESLA_LONG_LIMITS = {
    .max_accel = 425,       // 2 m/s^2
    .min_accel = 288,       // -3.48 m/s^2
    .inactive_accel = 375,  // 0. m/s^2
  };

  bool tx = true;
  bool violation = false;

  // Steering control: (0.1 * val) - 1638.35 in deg.
  if (!tesla_external_panda && (msg->addr == 0x488U)) {
    // We use 1/10 deg as a unit here
    int raw_angle_can = ((msg->data[0] & 0x7FU) << 8) | msg->data[1];
    int desired_angle = raw_angle_can - 16384;
    int steer_control_type = msg->data[2] >> 6;
    bool steer_control_enabled = steer_control_type == 1;  // ANGLE_CONTROL

    if (steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled, TESLA_STEERING_LIMITS, TESLA_LEGACY_STEERING_PARAMS)) {
      violation = true;
    }

    bool valid_steer_control_type = (steer_control_type == 0) ||  // NONE
                                    (steer_control_type == 1);    // ANGLE_CONTROL
    if (!valid_steer_control_type) {
      violation = true;
    }

    if (tesla_legacy_stock_lkas) {
      // Don't allow any steering commands when stock LKAS is active
      violation = true;
    }
  }

  // DAS_control: longitudinal control message
  if ((tesla_external_panda || tesla_hw1 || tesla_preap) && (msg->addr == das_control_msg)) {
    // No AEB events may be sent by openpilot
    int aeb_event = msg->data[2] & 0x03U;
    if (aeb_event != 0) {
      violation = true;
    }

    // Don't send long/cancel messages when the stock AEB system is active
    if (tesla_legacy_stock_aeb) {
      violation = true;
    }

    int raw_accel_max = ((msg->data[6] & 0x1FU) << 4) | (msg->data[5] >> 4);
    int raw_accel_min = ((msg->data[5] & 0x0FU) << 5) | (msg->data[4] >> 3);

    // Prevent both acceleration from being negative, as this could cause the car to reverse after coming to standstill
    if ((raw_accel_max < TESLA_LONG_LIMITS.inactive_accel) && (raw_accel_min < TESLA_LONG_LIMITS.inactive_accel)) {
      violation = true;
    }

    // Don't allow any acceleration limits above the safety limits
    violation |= longitudinal_accel_checks(raw_accel_max, TESLA_LONG_LIMITS);
    violation |= longitudinal_accel_checks(raw_accel_min, TESLA_LONG_LIMITS);
  }
  
  // Pedal Interceptor - TINKLA COMPATIBILITY
  // NOTE: Tinkla does NOT have any TX hook safety check for pedal (0x551).
  // They trust OpenPilot to handle engagement state. Adding restrictive checks
  // here causes pedal to be blocked before controls_allowed is set.
  // 
  // The pedal hardware has its own watchdog (counter validation) that will
  // disable if it stops receiving valid commands. OpenPilot handles engagement
  // state via latActive/longActive flags. Additional panda safety checks here
  // would only cause race conditions and prevent proper engagement.
  //
  // DO NOT ADD CONTROLS_ALLOWED CHECK HERE - it breaks pedal engagement timing.

  if (violation) {
    tx = false;
  }

  return tx;
}

static bool tesla_legacy_fwd_hook(int bus_num, int addr) {
  (void)bus_num;
  (void)addr;

  // Pre-AP has a single panda with nothing on bus 2.  Returning false (allow)
  // caused safety_fwd_hook to forward EVERY bus-0 message to bus 2, flooding
  // the dead TX queue (~1 300 overflows/s) and starving bus-0 TX interrupts
  // so openpilot's 0x488 / 0x214 never reached the wire.
  if (tesla_preap) {
    return true;   // block automatic forwarding
  }

  // HW1/HW2/HW3: allow default bus 0↔2 forwarding for chassis↔party CAN
  // bridging.  Selective relay-echo blocking is handled by check_relay in
  // the TX whitelist.
  return false;
}

static safety_config tesla_legacy_init(uint16_t param) {
  const int TESLA_FLAG_EXTERNAL_PANDA = 2;
  const int TESLA_FLAG_HW1 = 4;
  const int TESLA_FLAG_HW2 = 8;
  const int TESLA_FLAG_HW3 = 16;
  const int TESLA_FLAG_PREAP = 32;
  const int TESLA_FLAG_ENABLE_PEDAL = 64;
  const int TESLA_FLAG_RADAR_BEHIND_NOSECONE = 128;
  const int TESLA_FLAG_RADAR_EMULATION = 256;

  // Extract flags
  tesla_external_panda = GET_FLAG(param, TESLA_FLAG_EXTERNAL_PANDA);
  tesla_hw1 = GET_FLAG(param, TESLA_FLAG_HW1);
  tesla_hw2 = GET_FLAG(param, TESLA_FLAG_HW2);
  tesla_hw3 = GET_FLAG(param, TESLA_FLAG_HW3);
  tesla_preap = GET_FLAG(param, TESLA_FLAG_PREAP);
  tesla_enable_pedal = GET_FLAG(param, TESLA_FLAG_ENABLE_PEDAL);
  tesla_radar_behind_nosecone = GET_FLAG(param, TESLA_FLAG_RADAR_BEHIND_NOSECONE);
  tesla_radar_emulation = GET_FLAG(param, TESLA_FLAG_RADAR_EMULATION);

  if (tesla_radar_behind_nosecone) {
    radar_position = 1;
  }

  // Initialize state variables
  tesla_legacy_stock_aeb = false;
  tesla_legacy_stock_lkas = false;
  tesla_legacy_stock_lkas_prev = false;
  chassis_bus = 0U;
  di_torque1_msg = 0x106U;
  
  // Pre-AP state initialization
  tesla_gear = 4;  // Assume Drive initially
  tesla_gear_prev = 4;
  tesla_doors_open = false;
  pedal_can = -1;
  pedal_pressed = 0;
  tesla_radar_status = 0;
  tesla_last_radar_signal = 0;

  // Set DAS control message address
  das_control_msg = tesla_external_panda ? 0x2bfU : 0x2b9U;
  if (tesla_preap) das_control_msg = 0x2b9U;

  // Define message arrays (keeping them as is)
  static const CanMsg TESLA_TX_LEGACY_MSGS[] = {
    {0x488, 0, 4, .check_relay = true, .disable_static_blocking = true},  // DAS_steeringControl
    {0x27D, 0, 3, .check_relay = true, .disable_static_blocking = true},  // APS_eacMonitor
  };

  static const CanMsg TESLA_LEGACY_PT_MSGS[] = {
    {0x2bf, 0, 8, .check_relay = true, .disable_static_blocking = true},  // DAS_control
  };

  static const CanMsg TESLA_TX_LEGACY_HW1_MSGS[] = {
    {0x488, 0, 4, .check_relay = true, .disable_static_blocking = true},  // DAS_steeringControl
    {0x2b9, 0, 8, .check_relay = true, .disable_static_blocking = true},  // DAS_control
  };
  
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

  // Define RX check arrays (keeping them as is)
  static RxCheck tesla_legacy_pt_rx_checks[] = {
    {.msg = {{0x106, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},  // DI_torque1
    {.msg = {{0x1f8, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // BrakeMessage
    {.msg = {{0x2bf, 2, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DAS_control
    {.msg = {{0x256, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DI_state
  };

  static RxCheck tesla_legacy_hw1_rx_checks[] = {
    {.msg = {{0x108, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},  // DI_torque1
    {.msg = {{0x2b9, 2, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DAS_control
    {.msg = {{0x370, 0, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // EPAS_sysStatus (25hz)
    {.msg = {{0x155, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // ESP_private1
    {.msg = {{0x20a, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // BrakeMessage
    {.msg = {{0x368, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DI_state
    {.msg = {{0x488, 2, 4, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DAS_steeringControl
  };

  static RxCheck tesla_legacy_hw2_rx_checks[] = {
    {.msg = {{0x370, 0, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // EPAS_sysStatus (25hz)
    {.msg = {{0x155, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // ESP_private1
    {.msg = {{0x20a, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // BrakeMessage
    {.msg = {{0x368, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DI_state
    {.msg = {{0x488, 2, 4, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DAS_steeringControl
  };

  static RxCheck tesla_legacy_hw3_rx_checks[] = {
    {.msg = {{0x370, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // EPAS_sysStatus (100hz)
    {.msg = {{0x155, 1, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // ESP_private1
    {.msg = {{0x20a, 1, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // BrakeMessage
    {.msg = {{0x368, 1, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DI_state
    {.msg = {{0x488, 2, 4, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DAS_steeringControl
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

  // Determine configuration based on hardware type
  if (tesla_external_panda && (tesla_hw3 || tesla_hw2)) {
    return BUILD_SAFETY_CFG(tesla_legacy_pt_rx_checks, TESLA_LEGACY_PT_MSGS);
  }

  if (tesla_hw3) {
    chassis_bus = 1U;
    return BUILD_SAFETY_CFG(tesla_legacy_hw3_rx_checks, TESLA_TX_LEGACY_MSGS);
  }

  if (tesla_hw1) {
    di_torque1_msg = 0x108U;
    return BUILD_SAFETY_CFG(tesla_legacy_hw1_rx_checks, TESLA_TX_LEGACY_HW1_MSGS);
  }
  
  if (tesla_preap) {
    di_torque1_msg = 0x108U;
    return BUILD_SAFETY_CFG(tesla_preap_rx_checks, TESLA_TX_PREAP_MSGS);
  }

  // Default case: HW2
  return BUILD_SAFETY_CFG(tesla_legacy_hw2_rx_checks, TESLA_TX_LEGACY_MSGS);
}

const safety_hooks tesla_legacy_hooks = {
  .init = tesla_legacy_init,
  .rx = tesla_legacy_rx_hook,
  .tx = tesla_legacy_tx_hook,
  .fwd = tesla_legacy_fwd_hook,
};
