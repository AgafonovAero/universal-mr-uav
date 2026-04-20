#!/usr/bin/env bash
set -euo pipefail

ARDUPILOT_ROOT="${ARDUPILOT_ROOT:-$HOME/src/ardupilot}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --root)
            ARDUPILOT_ROOT="$2"
            shift 2
            ;;
        *)
            echo "Неизвестный параметр: $1"
            exit 1
            ;;
    esac
done

echo "Проверка ArduPilot внутри WSL"
echo "  целевой каталог: ${ARDUPILOT_ROOT}"

if command -v git >/dev/null 2>&1; then
    echo "  Git: да"
else
    echo "  Git: нет"
fi

if command -v python3 >/dev/null 2>&1 || command -v python >/dev/null 2>&1; then
    echo "  Python: да"
else
    echo "  Python: нет"
fi

if [[ -d "${ARDUPILOT_ROOT}" ]]; then
    echo "  ArduPilot: да"
else
    echo "  ArduPilot: нет"
fi

if [[ -f "${ARDUPILOT_ROOT}/Tools/autotest/sim_vehicle.py" ]]; then
    echo "  sim_vehicle.py: да"
else
    echo "  sim_vehicle.py: нет"
fi

exit 0
