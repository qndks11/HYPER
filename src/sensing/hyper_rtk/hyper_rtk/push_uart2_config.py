#!/usr/bin/env python3
"""Push UART2 moving-base RTK config to a u-blox ZED-F9P, every launch.

Why this exists
----------------
The moving-base heading pipeline needs base's UART2 configured to output
RTCM3 MSM7 (1077/1087/1097/1127) + 1230 + the u-blox-proprietary 4072.0
message, and rover's UART2 configured to accept RTCM3 input (README's
"u-center 사전 설정" section). u-center can set this, but these Ardusimple
boards have no backup battery -- unplugging the USB cable (e.g. to swap the
board over to u-center on a different machine, or just moving it between
hosts) drops power completely, and unless the config was saved with BOTH
BBR *and* Flash checked in UBX-CFG-CFG, it's gone the moment power returns.
That produced exactly this symptom: heading looks fine in u-center right
after configuring it, then reads as permanently invalid
(NavRELPOSNED9.flags stuck at FLAGS_GNSS_FIX_OK only, no DIFF_SOLN at all)
once the boards are plugged back in for ros2/hyper_rtk to run.

Rather than depend on that flash write surviving, this script re-pushes the
UART2 config over the board's own USB/UART1 control connection (the same
port ros2 later opens for the ublox_gps_node) every time rtk.launch.py
starts, using hand-built UBX-CFG-PRT / UBX-CFG-MSG frames -- no ROS message
dependency, no reliance on whatever is (or isn't) saved to flash. It must
run and fully close the serial port *before* ublox_gps_node opens the same
device (two processes reading the same port at once split the incoming
byte stream between them and corrupt both) -- rtk.launch.py chains that with
an OnProcessExit handler, not a fixed delay.

Because this goes over the *host* control link rather than UART2 itself, it
works regardless of whatever UART2 was left in -- that's what makes it able
to fix a UART2 config that stopped working entirely.

RTCM3 sub-message IDs below are u-blox's own numbering inside RTCM output
class 0xF5, confirmed against ublox_gps/config/c94_m8p_base.yaml's comments
(5, 87, 77, 230) plus the standard MSM7 +10 progression. Deliberately not
1005/1006 (static antenna reference position) -- moving base uses the
proprietary 4072.0 message instead, since the reference receiver isn't at a
surveyed fixed point.
"""

import argparse
import struct
import sys
import time

import serial

UBX_SYNC1 = 0xB5
UBX_SYNC2 = 0x62

CLASS_CFG = 0x06
ID_CFG_PRT = 0x00
ID_CFG_MSG = 0x01

CLASS_ACK = 0x05
ID_ACK_NAK = 0x00
ID_ACK_ACK = 0x01

CLASS_RTCM = 0xF5

PORT_ID_UART2 = 2

# CfgPRT.mode for 8N1 -- matches ublox_gps's own gps.cpp resetSerialPort()
# default (MODE_RESERVED1 | MODE_CHAR_LEN_8BIT | MODE_PARITY_NO | MODE_STOP_BITS_1).
MODE_8N1 = 0x8D0

PROTO_UBX = 1
PROTO_RTCM3 = 32

RTCM_IDS = {
    '1077': 77,     # GPS MSM7
    '1087': 87,     # GLONASS MSM7
    '1097': 97,     # Galileo MSM7
    '1127': 127,    # BeiDou MSM7
    '1230': 230,    # GLONASS code-phase biases
    '4072.0': 254,  # u-blox proprietary Reference Station PVT
}


def _checksum(body):
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def _frame(cls, msg_id, payload=b''):
    body = bytes([cls, msg_id]) + struct.pack('<H', len(payload)) + payload
    ck_a, ck_b = _checksum(body)
    return bytes([UBX_SYNC1, UBX_SYNC2]) + body + bytes([ck_a, ck_b])


def _cfg_prt_uart2(baud, in_proto, out_proto):
    payload = struct.pack(
        '<BBHIIHHHH',
        PORT_ID_UART2, 0, 0, MODE_8N1, baud,
        in_proto, out_proto, 0, 0)
    return _frame(CLASS_CFG, ID_CFG_PRT, payload)


def _cfg_msg_rate_uart2(msg_class, msg_id, rate):
    # Extended 8-byte CFG-MSG: msgClass, msgID, rate[DDC, UART1, UART2, USB, SPI, reserved].
    # Only UART2's slot is set -- explicitly zeroing the rest keeps this
    # message off USB/UART1/DDC/SPI regardless of whatever they had before.
    payload = bytes([msg_class, msg_id, 0, 0, rate, 0, 0, 0])
    return _frame(CLASS_CFG, ID_CFG_MSG, payload)


def _send_and_wait_ack(ser, frame, cls, msg_id, timeout, label):
    """Send one UBX config frame, wait for its ACK-ACK/ACK-NAK.

    The port is also carrying whatever the receiver is already streaming
    (NAV-PVT, NMEA, ...) since ublox_gps_node hasn't attached yet, so this
    has to pick the matching ACK out of a live mixed stream rather than
    assume the next frame in is the one we want.
    """
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    deadline = time.monotonic() + timeout
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if not chunk:
            continue
        buf += chunk
        while True:
            i = buf.find(bytes([UBX_SYNC1, UBX_SYNC2]))
            if i < 0:
                buf.clear()
                break
            if len(buf) < i + 6:
                del buf[:i]
                break
            length = struct.unpack('<H', buf[i + 4:i + 6])[0]
            end = i + 6 + length + 2
            if len(buf) < end:
                del buf[:i]
                break
            frame_cls, frame_id = buf[i + 2], buf[i + 3]
            got = bytes(buf[i:end])
            del buf[:end]
            if frame_cls == CLASS_ACK and frame_id in (ID_ACK_ACK, ID_ACK_NAK):
                acked_cls, acked_id = got[6], got[7]
                if acked_cls == cls and acked_id == msg_id:
                    ok = frame_id == ID_ACK_ACK
                    print(f'  {"ACK" if ok else "NAK"}  {label}')
                    return ok
    print(f'  TIMEOUT (no ACK/NAK)  {label}')
    return False


def push(device, role, open_baud, uart2_baud, timeout):
    print(f'{role}: pushing UART2 config to {device} '
          f'(control link @ {open_baud}, UART2 -> {uart2_baud} baud)')
    ok = True
    try:
        with serial.Serial(device, baudrate=open_baud, timeout=0.2) as ser:
            time.sleep(0.3)  # let the port settle before the first write
            if role == 'base':
                frame = _cfg_prt_uart2(uart2_baud, in_proto=0, out_proto=PROTO_RTCM3)
                ok &= _send_and_wait_ack(ser, frame, CLASS_CFG, ID_CFG_PRT, timeout,
                                          'CFG-PRT UART2 out=RTCM3X')
                for name, msg_id in RTCM_IDS.items():
                    frame = _cfg_msg_rate_uart2(CLASS_RTCM, msg_id, 1)
                    ok &= _send_and_wait_ack(ser, frame, CLASS_CFG, ID_CFG_MSG, timeout,
                                              f'CFG-MSG RTCM {name} rate=1 on UART2')
            else:
                frame = _cfg_prt_uart2(uart2_baud, in_proto=PROTO_RTCM3 | PROTO_UBX, out_proto=0)
                ok &= _send_and_wait_ack(ser, frame, CLASS_CFG, ID_CFG_PRT, timeout,
                                          'CFG-PRT UART2 in=RTCM3X')
    except serial.SerialException as exc:
        print(f'{role}: could not open {device}: {exc}')
        return False
    print(f'{role}: {"OK -- UART2 config applied" if ok else "one or more steps FAILED, see above"}')
    return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', required=True, help='serial device, e.g. /dev/tty_ublox_base')
    parser.add_argument('--role', required=True, choices=['base', 'rover'])
    parser.add_argument('--open-baud', type=int, default=115200,
                         help='baud for the control connection (USB CDC ignores this; '
                              'matches common_params.uart1.baudrate in rtk.launch.py)')
    parser.add_argument('--uart2-baud', type=int, default=115200,
                         help='baud to set UART2 to -- must match on base and rover')
    parser.add_argument('--timeout', type=float, default=1.0, help='ACK wait timeout, seconds')
    args = parser.parse_args()

    ok = push(args.device, args.role, args.open_baud, args.uart2_baud, args.timeout)
    # Non-zero exit is informational only (rtk.launch.py starts ublox_gps_node
    # either way) -- a NAK/timeout here usually means UART2 wiring itself is
    # the problem, which no amount of retrying this script will fix.
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
