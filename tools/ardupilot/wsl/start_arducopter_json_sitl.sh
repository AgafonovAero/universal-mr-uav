#!/usr/bin/env bash
set -euo pipefail

ARDUPILOT_ROOT="${ARDUPILOT_ROOT:-$HOME/src/ardupilot}"
IP_ADDRESS="127.0.0.1"
VEHICLE="ArduCopter"
FRAME="quad"
USE_CONSOLE=1
USE_MAP=1

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-console)
            USE_CONSOLE=0
            shift
            ;;
        --no-map)
            USE_MAP=0
            shift
            ;;
        --ip)
            IP_ADDRESS="$2"
            shift 2
            ;;
        --vehicle)
            VEHICLE="$2"
            shift 2
            ;;
        --frame)
            FRAME="$2"
            shift 2
            ;;
        *)
            echo "Неизвестный параметр: $1"
            exit 1
            ;;
    esac
done

if [[ ! -d "$ARDUPILOT_ROOT" ]]; then
    echo "Ошибка: каталог ArduPilot не найден: ${ARDUPILOT_ROOT}"
    exit 1
fi

cd "$ARDUPILOT_ROOT"

if [[ ! -f "Tools/autotest/sim_vehicle.py" ]]; then
    echo "Ошибка: sim_vehicle.py не найден в ${ARDUPILOT_ROOT}/Tools/autotest/."
    exit 1
fi

COMMAND=(Tools/autotest/sim_vehicle.py -v "$VEHICLE" -f "$FRAME" --model "JSON:${IP_ADDRESS}")

if [[ "$USE_CONSOLE" -eq 1 ]]; then
    COMMAND+=(--console)
fi

if [[ "$USE_MAP" -eq 1 ]]; then
    COMMAND+=(--map)
fi

echo "Запуск ArduPilot SITL в режиме JSON"
echo "  каталог ArduPilot: ${ARDUPILOT_ROOT}"
echo "  команда: ${COMMAND[*]}"

exec "${COMMAND[@]}"
