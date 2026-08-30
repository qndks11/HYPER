# patches

`deps.repos`로 가져오는 외부 저장소에 적용할 패치들입니다. 그 저장소들은 HYPER git이
추적하지 않으므로, 거기서 직접 고친 내용은 다음 `vcs import` 때 사라집니다. 그래서 변경분을
여기 패치로 남겨 둡니다 (같은 이유가 `sensors.launch.py` 상단 주석에도 적혀 있습니다).
