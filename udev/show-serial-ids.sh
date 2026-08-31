#!/usr/bin/env bash
# 지금 꽂혀 있는 USB 시리얼 장치마다 udev 규칙에 필요한 값을 뽑는다.
# 99-hyper-serial.rules를 이 차에 맞춰 채울 때 쓴다.
#
#   ./udev/show-serial-ids.sh
#
# 출력 열:
#   device     /dev/ttyUSB0 같은 커널 이름
#   idVendor / idProduct   ATTRS{idVendor} / ATTRS{idProduct}에 그대로 넣는 값
#   serial     ATTRS{serial} -- 비어 있으면(CH340이 대개 그렇다) 칩이 겹칠 때
#              KERNELS(USB port)로 구분해야 한다
#   USB port   KERNELS== 에 넣는 물리 포트 경로
#   by-id      /dev/serial/by-id 심볼릭 링크(있으면). 규칙 없이 바로 써도 되는 경로다
set -u

shopt -s nullglob
devices=(/dev/ttyUSB* /dev/ttyACM*)
if [ ${#devices[@]} -eq 0 ]; then
    echo "USB 시리얼 장치가 하나도 없습니다 (/dev/ttyUSB*, /dev/ttyACM*)."
    exit 1
fi

if ! command -v udevadm >/dev/null 2>&1; then
    echo "udevadm이 없습니다 -- 실차 PC(Ubuntu)에서 실행하세요."
    exit 1
fi

printf '%-14s %-9s %-10s %-18s %-12s %s\n' \
    device idVendor idProduct serial "USB port" by-id

for dev in "${devices[@]}"; do
    info=$(udevadm info -a -n "$dev" 2>/dev/null)

    # -a 출력은 장치에서 루트로 올라가며 부모를 나열한다. 각 속성의 첫 등장이
    # 가장 가까운 부모(= 이 장치의 USB 인터페이스)의 값이다.
    vendor=$(grep -m1 'ATTRS{idVendor}' <<<"$info" | cut -d'"' -f2)
    product=$(grep -m1 'ATTRS{idProduct}' <<<"$info" | cut -d'"' -f2)
    serial=$(grep -m1 'ATTRS{serial}' <<<"$info" | cut -d'"' -f2)
    # KERNELS는 idVendor를 가진 블록(USB 장치)의 것을 써야 한다.
    port=$(awk '/ATTRS\{idVendor\}/ {print prev; exit} /KERNELS==/ {prev=$0}' <<<"$info" \
           | cut -d'"' -f2)

    byid=''
    for link in /dev/serial/by-id/*; do
        if [ "$(readlink -f "$link")" = "$(readlink -f "$dev")" ]; then
            byid=$(basename "$link")
            break
        fi
    done

    printf '%-14s %-9s %-10s %-18s %-12s %s\n' \
        "$dev" "${vendor:--}" "${product:--}" "${serial:--}" "${port:--}" "${byid:--}"
done

cat <<'EOF'

위 값을 udev/99-hyper-serial.rules에 채운 뒤:
  sudo cp udev/99-hyper-serial.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ls -l /dev/tty_Ardusimple /dev/tty_ebimu /dev/tty_arduino
EOF
