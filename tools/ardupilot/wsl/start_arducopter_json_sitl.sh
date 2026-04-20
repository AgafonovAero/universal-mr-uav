#!/usr/bin/env bash
set -euo pipefail

EXECUTE=0
ARDUPILOT_ROOT="${ARDUPILOT_ROOT:-$HOME/src/ardupilot}"
IP_ADDRESS="127.0.0.1"
VEHICLE="ArduCopter"
FRAME="quad"
MAVLINK_PORT="14550"
SECONDARY_MAVLINK_PORT="14552"
USE_CONSOLE=1
USE_MAP=1

resolve_windows_host_ip() {
    local gateway_ip

    gateway_ip="$(ip -4 route list default | awk '{print $3; exit}')"
    if [[ -n "${gateway_ip}" ]]; then
        printf '%s' "${gateway_ip}"
        return 0
    fi

    return 1
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --execute)
            EXECUTE=1
            shift
            ;;
        --dry-run)
            EXECUTE=0
            shift
            ;;
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
        --mavlink-port)
            MAVLINK_PORT="$2"
            shift 2
            ;;
        --secondary-mavlink-port)
            SECONDARY_MAVLINK_PORT="$2"
            shift 2
            ;;
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

if [[ ! -d "${ARDUPILOT_ROOT}" ]]; then
    echo "Ошибка: каталог ArduPilot не найден: ${ARDUPILOT_ROOT}"
    exit 1
fi

cd "${ARDUPILOT_ROOT}"

if [[ ! -f "Tools/autotest/sim_vehicle.py" ]]; then
    echo "Ошибка: sim_vehicle.py не найден в ${ARDUPILOT_ROOT}/Tools/autotest/."
    exit 1
fi

if [[ ! -x "build/sitl/bin/arducopter" ]]; then
    echo "Ошибка: исполняемый файл ArduCopter SITL не найден: ${ARDUPILOT_ROOT}/build/sitl/bin/arducopter"
    echo "Для запуска необходимо предварительно собрать ArduPilot внутри WSL."
    exit 1
fi

if [[ "${IP_ADDRESS}" == "127.0.0.1" || "${IP_ADDRESS}" == "localhost" ]]; then
    RESOLVED_IP="$(resolve_windows_host_ip || true)"
    if [[ -n "${RESOLVED_IP}" ]]; then
        IP_ADDRESS="${RESOLVED_IP}"
    fi
fi

if [[ ! -f "Tools/autotest/default_params/copter.parm" ]]; then
    echo "Ошибка: файл параметров ArduCopter не найден: ${ARDUPILOT_ROOT}/Tools/autotest/default_params/copter.parm"
    exit 1
fi

cd "${ARDUPILOT_ROOT}/ArduCopter"

COMMAND=(
    ../build/sitl/bin/arducopter
    --model "JSON:${IP_ADDRESS}"
    --speedup 1
    --slave 0
    --serial0="udpclient:${IP_ADDRESS}:${MAVLINK_PORT}"
    --defaults ../Tools/autotest/default_params/copter.parm
    --sim-address="${IP_ADDRESS}"
    -I0
)

if [[ -n "${SECONDARY_MAVLINK_PORT}" && "${SECONDARY_MAVLINK_PORT}" != "0" ]]; then
    COMMAND+=(
        --serial1="udpclient:${IP_ADDRESS}:${SECONDARY_MAVLINK_PORT}"
    )
fi

echo "Запуск ArduPilot SITL в режиме JSON"
echo "  каталог ArduPilot: ${ARDUPILOT_ROOT}"
echo "  транспорт JSON/UDP к MATLAB: ${IP_ADDRESS}:9002"
echo "  поток MAVLink UDP к Windows: ${IP_ADDRESS}:${MAVLINK_PORT}"
if [[ -n "${SECONDARY_MAVLINK_PORT}" && "${SECONDARY_MAVLINK_PORT}" != "0" ]]; then
    echo "  дополнительный поток MAVLink: ${IP_ADDRESS}:${SECONDARY_MAVLINK_PORT}"
fi
echo "  тип аппарата: ${VEHICLE}"
echo "  схема аппарата: ${FRAME}"

if [[ "${USE_CONSOLE}" -eq 0 ]]; then
    echo "  встроенная консоль SITL: отключена параметром сценария"
fi

if [[ "${USE_MAP}" -eq 0 ]]; then
    echo "  графическая карта SITL: отключена параметром сценария"
fi

echo "  команда: ${COMMAND[*]}"

if [[ "${EXECUTE}" -eq 1 ]]; then
    exec "${COMMAND[@]}"
else
    echo "Режим проверки: команда не выполнялась."
fi
