#!/usr/bin/env bash
set -euo pipefail

SOURCE_DIR="$(cd "$(dirname "$0")" && pwd)"
TARGET_DIR="${HOME}/HYPER/src/parking_cpp"

rm -rf "${TARGET_DIR}"
cp -a "${SOURCE_DIR}" "${TARGET_DIR}"
rm -f "${TARGET_DIR}/copy_to_hyper.sh"

echo "Copied to ${TARGET_DIR}"
echo "Next: cd ~/HYPER && colcon build --packages-select parking_cpp --symlink-install"
