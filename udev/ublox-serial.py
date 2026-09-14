#!/usr/bin/env python3
"""u-blox ZED-F9P의 USB 시리얼 문자열(iSerialNumber)을 읽고 쓴다.

두 보드가 idVendor/idProduct까지 같아서(1546:01a9) 공장 상태로는 udev가 구분하지
못한다. CFG-USB-SERIAL_NO_STR0..3에 문자열을 써서 Flash에 저장하면 재열거 후
ATTRS{serial}로 잡히고, 99-hyper-serial.rules가 그 값으로 심볼릭 링크를 만든다.

  ./ublox-serial.py --list
  ./ublox-serial.py /dev/ttyACM0 HYPER-GNSS-BASE --reset

칩 ID(UBX-SEC-UNIQID)는 실리콘 고유값이라 시리얼이 비어 있어도 보드를 식별한다.
"""
import argparse, glob, os, sys, time

KEYS = [0x50650015, 0x50650016, 0x50650017, 0x50650018]  # CFG-USB-SERIAL_NO_STR0..3
MAX_LEN = 32  # 8바이트 x 4 키


def _ck(b):
    a = c = 0
    for x in b:
        a = (a + x) & 0xFF
        c = (c + a) & 0xFF
    return bytes([a, c])


def _frame(cls, mid, pl=b''):
    body = bytes([cls, mid, len(pl) & 0xFF, len(pl) >> 8]) + pl
    return b'\xb5\x62' + body + _ck(body)


def _exchange(fd, pkt, cls, mid, timeout=3.0):
    """('msg', payload) | ('ack',) | ('nak',) | None"""
    os.write(fd, pkt)
    buf = b''
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            buf += os.read(fd, 4096)
        except BlockingIOError:
            time.sleep(0.05)
            continue
        i = 0
        while True:
            i = buf.find(b'\xb5\x62', i)
            if i < 0 or len(buf) < i + 6:
                break
            c, m = buf[i + 2], buf[i + 3]
            ln = buf[i + 4] | (buf[i + 5] << 8)
            if len(buf) < i + 6 + ln + 2:
                break
            p = buf[i + 6:i + 6 + ln]
            if c == cls and m == mid:
                return ('msg', p)
            if c == 0x05 and ln >= 2 and p[0] == cls and p[1] == mid:
                return ('nak',) if m == 0x00 else ('ack',)
            i += 2
    return None


def _open(dev):
    # 커널이 canonical 모드로 열어 두면 UBX 바이너리가 깨져서 응답을 못 받는다.
    os.system('stty -F %s raw -echo 115200' % dev)
    return os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)


def chip_id(fd):
    r = _exchange(fd, _frame(0x27, 0x03), 0x27, 0x03)
    return r[1][4:9].hex(':') if r and r[0] == 'msg' and len(r[1]) >= 9 else None


def read_serial(fd, layer=0):
    q = b''.join(k.to_bytes(4, 'little') for k in KEYS)
    r = _exchange(fd, _frame(0x06, 0x8B, bytes([0x00, layer, 0, 0]) + q), 0x06, 0x8B)
    if not r or r[0] != 'msg':
        return None
    out, p = b'', r[1][4:]
    while len(p) >= 12:
        out += p[4:12]
        p = p[12:]
    return out.split(b'\0')[0].decode('ascii', 'replace')


def write_serial(fd, name):
    s = name.encode('ascii')
    if len(s) > MAX_LEN:
        sys.exit('시리얼은 최대 %d바이트다: %r (%d)' % (MAX_LEN, name, len(s)))
    s = s.ljust(MAX_LEN, b'\0')
    pairs = b''.join(k.to_bytes(4, 'little') + s[i * 8:i * 8 + 8] for i, k in enumerate(KEYS))
    # Flash가 없는 보드도 있어서 RAM+BBR로 한 번 더 시도한다.
    for layers, label in ((0x07, 'RAM+BBR+Flash'), (0x03, 'RAM+BBR')):
        r = _exchange(fd, _frame(0x06, 0x8A, bytes([0x00, layers, 0, 0]) + pairs), 0x06, 0x8A)
        print('  VALSET %-14s -> %s' % (label, r[0] if r else 'timeout'))
        if r and r[0] == 'ack':
            return True
    return False


def reset(fd):
    # navBbrMask=0x0000 (hot start, 항법 데이터 유지) + resetMode=0x00 (하드웨어 리셋)
    os.write(fd, _frame(0x06, 0x04, bytes([0x00, 0x00, 0x00, 0x00])))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('device', nargs='?', help='예: /dev/ttyACM0')
    ap.add_argument('serial', nargs='?', help='예: HYPER-GNSS-BASE')
    ap.add_argument('--list', action='store_true', help='꽂힌 보드를 모두 훑어본다')
    ap.add_argument('--reset', action='store_true',
                    help='쓴 뒤 CFG-RST로 재열거한다 (안 하면 뺐다 꽂아야 반영된다)')
    a = ap.parse_args()

    if a.list or not a.device:
        for dev in sorted(glob.glob('/dev/ttyACM*')):
            fd = _open(dev)
            try:
                cid = chip_id(fd)
                if cid is None:
                    continue  # u-blox가 아니거나 응답 없음
                print('%s  chip=%s  serial=%r' % (dev, cid, read_serial(fd) or ''))
            finally:
                os.close(fd)
        return

    if not a.serial:
        ap.error('시리얼 문자열을 같이 줘야 한다 (읽기만 하려면 --list)')

    fd = _open(a.device)
    try:
        print('%s  chip=%s  현재 serial=%r' % (a.device, chip_id(fd), read_serial(fd) or ''))
        if not write_serial(fd, a.serial):
            sys.exit('VALSET 실패')
        print('  VALGET Flash -> %r' % read_serial(fd, layer=2))
        if a.reset:
            reset(fd)
            print('  CFG-RST 보냄 -- 몇 초 뒤 재열거된다')
    finally:
        os.close(fd)
    print('확인: udevadm info -q property -n <dev> | grep ID_SERIAL_SHORT')


if __name__ == '__main__':
    main()
