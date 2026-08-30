# patches

`deps.repos`로 가져오는 외부 저장소에 적용할 패치들입니다. 그 저장소들은 HYPER git이
추적하지 않으므로, 거기서 직접 고친 내용은 다음 `vcs import` 때 사라집니다. 그래서 변경분을
여기 패치로 남겨 둡니다 (같은 이유가 `sensors.launch.py` 상단 주석에도 적혀 있습니다).

## witmotion_ros2-ble-reliability.patch

WT901BLE가 `real.launch.py`에서 자주 연결에 실패하던 문제의 드라이버 쪽 수정입니다.
`qndks11/witmotion_ros2` `master`에 그대로 적용됩니다.

- 스캔을 `scan_for()`(타임아웃 전체를 무조건 블로킹) 대신 직접 돌려 **장치를 찾는 즉시**,
  그리고 종료 신호를 받는 즉시 빠져나옵니다.
- 모든 대기를 종료 가능하게 바꿔, Ctrl-C 시 최대 15초 걸리던 종료가 즉시 끝납니다.
  → 소멸자의 `disconnect()`가 실제로 실행되므로 **BLE 연결이 남지 않습니다**. 이게 "한 번
  띄웠다 내리면 그 다음부터 안 붙는" 증상의 근본 원인이었습니다.
- `connect()` 실패 시 전체 재스캔(15초) 대신 제자리에서 3회 재시도합니다. BlueZ는 discovery를
  멈춘 직후 첫 connect를 자주 abort합니다.
- 이미 연결이 남아 있으면 끊고 새로 붙습니다.
- `on_scan_updated`에도 같은 매처를 겁니다. BlueZ가 이미 아는 장치는 `on_scan_found`가 아니라
  속성 업데이트로만 올라옵니다.
- notify 콜백을 try/catch로 감쌉니다. 이 콜백은 SimpleBLE 내부 스레드에서 돌기 때문에
  예외가 새어 나가면 `std::terminate`로 프로세스가 통째로 죽습니다.
- 스캔 콜백이 스택 지역변수를 참조하던 use-after-free 가능성을 없앴습니다.
- `device_address`로 필터링해 놓고 실패 로그에는 `device_name`을 찍던 것을 고쳤습니다.

### 적용

```bash
cd ~/HYPER/src/sensing/witmotion_ros2
git apply ~/HYPER/patches/witmotion_ros2-ble-reliability.patch
cd ~/HYPER && colcon build --packages-select witmotion_ros2 && source install/setup.bash
```

업스트림(`qndks11/witmotion_ros2`)에 머지되면 이 패치와 이 항목을 지우세요.
