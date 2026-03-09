import struct
import crcmod
from ctypes import create_string_buffer
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import V_CRUISE_MAX
from opendbc.car.tesla.values import CANBUS, CarControllerParams

# ============================================
# Comma Pedal Constants (from Tinkla teslacan.py)
# ============================================
# These are the scaling/offset values for the Comma Pedal protocol
# Used to convert pedal voltage to raw CAN values
PEDAL_M1 = 0.050796813    # Primary scaling factor
PEDAL_M2 = 0.101593626    # Secondary scaling factor (2x M1 for redundancy)
PEDAL_D = -22.85856576    # Offset

# CAN Message IDs
GAS_COMMAND_ID = 0x551    # 1361 - Command to Comma Pedal
GAS_SENSOR_ID = 0x552     # 1362 - Feedback from Comma Pedal


class TeslaCANRaven:
  def __init__(self, packers):
    self.packers = packers

  @staticmethod
  def checksum(msg_id, dat):
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_steering_control(self, counter, angle, enabled):
    values = {
      "DAS_steeringControlCounter": counter,
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,
    }

    data = self.packers[CANBUS.party].make_can_msg("DAS_steeringControl", CANBUS.party, values)[1]
    values["DAS_steeringControlChecksum"] = self.checksum(0x488, data[:3])
    return self.packers[CANBUS.party].make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active):
    set_speed = max(v_ego * CV.MS_TO_KPH, 0)
    if active:
      # TODO: this causes jerking after gas override when above set speed
      set_speed = 0 if accel < 0 else V_CRUISE_MAX

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": counter,
    }

    data = self.packers[CANBUS.powertrain].make_can_msg("DAS_control", CANBUS.powertrain, values)[1]
    values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
    return self.packers[CANBUS.powertrain].make_can_msg("DAS_control", CANBUS.powertrain, values)

  def create_steering_allowed(self, counter):
    values = {
      "APS_eacMonitorCounter": counter,
      "APS_eacAllow": 1,
    }

    data = self.packers[CANBUS.party].make_can_msg("APS_eacMonitor", CANBUS.party, values)[1]
    values["APS_eacMonitorChecksum"] = self.checksum(0x27d, data[:2])
    return self.packers[CANBUS.party].make_can_msg("APS_eacMonitor", CANBUS.party, values)


class TeslaCANPreAP(TeslaCANRaven):
  def __init__(self, packers, pedal_packer):
    super().__init__(packers)
    self.pedal_packer = pedal_packer
    # Pedal CAN bus: 2 by default, can be 0 if configured
    # This will be set by carcontroller based on nap_conf
    self.pedal_can_bus = 2
    # CRITICAL: Dedicated pedal counter that increments with each message sent
    # This is required by the pedal firmware's watchdog - it expects consecutive counters
    # If counter doesn't increment, the firmware rejects the message (FAULT_BAD_CHECKSUM or timeout)
    self.pedal_idx = 0
    # CRC-8 for STW_ACTN_RQ (Tinkla teslacan.py line 18-19)
    # Polynomial 0x1D, init=0x00, xorOut=0xFF — NOT the simple byte-sum checksum
    self.stw_crc = crcmod.mkCrcFun(0x11d, initCrc=0x00, rev=False, xorOut=0xff)

  @staticmethod
  def pedal_checksum(msg_id: int, dat: bytes) -> int:
    """
    Calculate checksum for Comma Pedal GAS_COMMAND message.
    
    This is the EXACT algorithm from Tinkla's panda/board/pedal/main.c:
    
      uint8_t pedal_checksum(uint8_t *dat, int len, int addr) {
        int i;
        uint8_t s = 0;
        s += ((addr)&0xFF) + ((addr>>8)&0xFF);
        for (i = 0; i < len; i++) {
          s = (s + dat[i]) & 0xFF;
        }
        return s;
      }
    
    The firmware calls: pedal_checksum(dat, 5, 0x551)
    Which sums: addr_low + addr_high + dat[0] + dat[1] + dat[2] + dat[3] + dat[4]
    
    In Python, we pass the full 6-byte buffer where byte[5] is 0 at calculation time.
    sum(dat) = sum(dat[0:5]) + 0 = sum(dat[0:5]), which is what firmware expects.
    
    Args:
      msg_id: CAN message ID (0x551 for pedal command)
      dat: Message bytes (6 bytes, with byte 5 being 0 before checksum is written)
      
    Returns:
      8-bit checksum value
    """
    # Split address into low and high bytes
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    # Sum all data bytes (byte 5 will be 0 at this point)
    ret += sum(dat)
    return ret & 0xFF

  def create_pedal_command(self, accel_command: float, enable: int = 1, pedal_can_bus: int = None):
    """
    Create GAS_COMMAND (0x551) message to Comma Pedal.
    
    BYTE-FOR-BYTE COMPATIBLE with Tinkla's teslacan.py create_pedal_command_msg().
    Uses raw struct packing to ensure exact binary compatibility.
    
    CRITICAL: This function manages its own rolling counter (self.pedal_idx) that
    increments with each call. The pedal firmware REQUIRES consecutive counter values
    or it will reject the message with FAULT_BAD_CHECKSUM or go into timeout.
    
    Message Format (6 bytes):
      Byte 0: GAS_COMMAND high byte (MSB)
      Byte 1: GAS_COMMAND low byte (LSB)  
      Byte 2: GAS_COMMAND2 high byte (MSB) - redundant value for safety
      Byte 3: GAS_COMMAND2 low byte (LSB)
      Byte 4: [Enable:1][Reserved:3][Counter:4] = (enable << 7) | (idx & 0x0F)
      Byte 5: Checksum
    
    Args:
      accel_command: Pedal voltage value (from calibration transform via nap_conf.di_to_pedal())
      enable: 1 to enable pedal actuation, 0 to disable (idle/coast)
      pedal_can_bus: CAN bus for pedal (0 or 2), defaults to self.pedal_can_bus
      
    Returns:
      CAN message tuple: (msg_id, data_bytes, bus)
    """
    msg_id = GAS_COMMAND_ID  # 0x551
    msg_len = 6
    
    if pedal_can_bus is None:
      pedal_can_bus = self.pedal_can_bus
    
    # Get current counter value and increment for next call
    # This is how Tinkla does it in PCC_module.py
    idx = self.pedal_idx
    self.pedal_idx = (self.pedal_idx + 1) % 16
    
    # Apply Tinkla's encoding formula from teslacan.py:
    #   m1 = 0.050796813  (primary scaling)
    #   m2 = 0.101593626  (secondary scaling = 2 * m1 for redundancy check)
    #   d = -22.85856576  (offset)
    #   int_accelCommand = int((accelCommand - d) / m1)
    #   int_accelCommand2 = int((accelCommand - d) / m2)
    if enable == 1:
      int_accel_command = int((accel_command - PEDAL_D) / PEDAL_M1)
      int_accel_command2 = int((accel_command - PEDAL_D) / PEDAL_M2)
    else:
      # When disabled, send zero values (Tinkla behavior)
      int_accel_command = 0
      int_accel_command2 = 0
    
    # Clip to valid 16-bit unsigned range (same as Tinkla's clip(val, 0, 65534))
    int_accel_command = max(0, min(65534, int_accel_command))
    int_accel_command2 = max(0, min(65534, int_accel_command2))
    
    # Pack message bytes using EXACT Tinkla format
    # From teslacan.py create_pedal_command_msg():
    #   struct.pack_into(
    #     "BBBBB", msg, 0,
    #     int((int_accelCommand >> 8) & 0xFF),  # Byte 0: cmd1 high
    #     int_accelCommand & 0xFF,               # Byte 1: cmd1 low
    #     int((int_accelCommand2 >> 8) & 0xFF), # Byte 2: cmd2 high
    #     int_accelCommand2 & 0xFF,              # Byte 3: cmd2 low
    #     ((enable << 7) + idx) & 0xFF,          # Byte 4: enable|counter
    #   )
    msg = create_string_buffer(msg_len)
    struct.pack_into(
      "BBBBB",
      msg,
      0,
      int((int_accel_command >> 8) & 0xFF),
      int(int_accel_command & 0xFF),
      int((int_accel_command2 >> 8) & 0xFF),
      int(int_accel_command2 & 0xFF),
      int(((enable << 7) + idx) & 0xFF),
    )
    
    # Calculate and append checksum (Tinkla: self.checksum(msg_id, msg.raw))
    # At this point msg.raw[5] is 0x00, so sum includes bytes 0-4 only
    checksum = self.pedal_checksum(msg_id, msg.raw)
    struct.pack_into("B", msg, msg_len - 1, checksum)
    
    # Return format: (address, data_bytes, bus)
    # This matches openpilot's make_can_msg() and can_list_to_can_capnp() expectations
    return (msg_id, bytes(msg.raw), pedal_can_bus)

  def get_pedal_idx(self) -> int:
    """Return current pedal counter value (for debugging/logging)."""
    return self.pedal_idx

  def reset_pedal_idx(self):
    """Reset pedal counter to 0 (use when recovering from fault state)."""
    self.pedal_idx = 0

  def create_epas_control(self, counter, mode):
    # EPB_epasControl (0x214 / 532) - Ported from Tinkla safety_tesla.h do_EPB_epasControl()
    # This message tells EPAS to allow EAC (Electronic Angle Control)
    # Tinkla sends: MLB = 0x01 + (counter << 8) + ((0x17 + counter) << 16)
    # Byte 0: 0x01 (EPB_epasEACAllow value)
    # Byte 1: counter (0-15)
    # Byte 2: checksum (0x17 + counter = base checksum + counter)
    values = {
      "EPB_epasEACAllow": mode,  # 1 = allow angle control
      "EPB_epasControlCounter": counter,
      "EPB_epasControlChecksum": 0,
    }

    data = self.packers[CANBUS.party].make_can_msg("EPB_epasControl", CANBUS.party, values)[1]
    # Checksum is calculated using ID 0x214 (532 decimal)
    values["EPB_epasControlChecksum"] = self.checksum(0x214, data)
    return self.packers[CANBUS.party].make_can_msg("EPB_epasControl", CANBUS.party, values)

  def create_action_request(self, button_to_press, bus, counter, msg_stw=None):
    """
    Create STW_ACTN_RQ message to simulate cruise stalk button press.
    
    Ported from Tinkla teslacan.py create_action_request().
    Used for cruise button spam fallback when no pedal is installed.
    
    Args:
      button_to_press: CruiseButtons value (RES_ACCEL, DECEL_SET, etc.)
      bus: CAN bus number
      counter: Message counter (0-15)
      msg_stw: Original STW_ACTN_RQ message dict (optional, for preserving other signals)
      
    Returns:
      CAN message tuple
    """
    # STW_ACTN_RQ (0x45 / 69)
    # From DBC: VAL_ 69 SpdCtrlLvr_Stat 32 "DN_1ST" 16 "UP_1ST" 8 "DN_2ND" 4 "UP_2ND" 2 "RWD" 1 "FWD" 0 "IDLE"
    
    if msg_stw is not None:
      # Preserve original message values, just change the button
      values = {
        "MC_STW_ACTN_RQ": counter,
        "CRC_STW_ACTN_RQ": 0,  # Will be recalculated
        "SpdCtrlLvr_Stat": button_to_press,
        "VSL_Enbl_Stat": msg_stw.get("VSL_Enbl_Stat", 0),
        "DTR_Dist_Rq": msg_stw.get("DTR_Dist_Rq", 0),
        "TurnIndLvr_Stat": msg_stw.get("TurnIndLvr_Stat", 0),
        "HiBmLvr_Stat": msg_stw.get("HiBmLvr_Stat", 0),
        "WprWashSw_Psd": msg_stw.get("WprWashSw_Psd", 0),
        "WprWash_R_Sw_Posn_V2": msg_stw.get("WprWash_R_Sw_Posn_V2", 0),
        "StW_Lvr_Stat": msg_stw.get("StW_Lvr_Stat", 0),
        "StW_Cond_Flt": msg_stw.get("StW_Cond_Flt", 0),
        "StW_Cond_Psd": msg_stw.get("StW_Cond_Psd", 0),
        "HrnSw_Psd": msg_stw.get("HrnSw_Psd", 0),
        "StW_Sw00_Psd": msg_stw.get("StW_Sw00_Psd", 0),
        "StW_Sw01_Psd": msg_stw.get("StW_Sw01_Psd", 0),
        "StW_Sw02_Psd": msg_stw.get("StW_Sw02_Psd", 0),
        "StW_Sw03_Psd": msg_stw.get("StW_Sw03_Psd", 0),
        "StW_Sw04_Psd": msg_stw.get("StW_Sw04_Psd", 0),
        "StW_Sw05_Psd": msg_stw.get("StW_Sw05_Psd", 0),
        "StW_Sw06_Psd": msg_stw.get("StW_Sw06_Psd", 0),
      }
    else:
      # Minimal message with just the button press
      values = {
        "MC_STW_ACTN_RQ": counter,
        "CRC_STW_ACTN_RQ": 0,
        "SpdCtrlLvr_Stat": button_to_press,
        "VSL_Enbl_Stat": 0,
        "DTR_Dist_Rq": 0,
        "TurnIndLvr_Stat": 0,
        "HiBmLvr_Stat": 0,
        "WprWashSw_Psd": 0,
        "WprWash_R_Sw_Posn_V2": 0,
        "StW_Lvr_Stat": 0,
        "StW_Cond_Flt": 0,
        "StW_Cond_Psd": 0,
        "HrnSw_Psd": 0,
        "StW_Sw00_Psd": 0,
        "StW_Sw01_Psd": 0,
        "StW_Sw02_Psd": 0,
        "StW_Sw03_Psd": 0,
        "StW_Sw04_Psd": 0,
        "StW_Sw05_Psd": 0,
        "StW_Sw06_Psd": 0,
      }
    
    data = self.packers[CANBUS.party].make_can_msg("STW_ACTN_RQ", bus, values)[1]
    # STW_ACTN_RQ uses CRC-8 polynomial (Tinkla teslacan.py line 308), NOT byte-sum checksum
    values["CRC_STW_ACTN_RQ"] = self.stw_crc(data[:7])
    return self.packers[CANBUS.party].make_can_msg("STW_ACTN_RQ", bus, values)
