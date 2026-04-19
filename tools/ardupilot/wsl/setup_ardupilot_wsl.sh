#!/usr/bin/env bash
set -euo pipefail

EXECUTE=0
ARDUPILOT_ROOT="${ARDUPILOT_ROOT:-$HOME/src/ardupilot}"
REPORT_PATH="${HOME}/ardupilot_setup_report.txt"

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

mkdir -p "$(dirname "$REPORT_PATH")"
exec > >(tee "$REPORT_PATH") 2>&1

echo "Подготовка внешнего каталога ArduPilot внутри WSL"
echo "  целевой каталог: ${ARDUPILOT_ROOT}"
echo "  режим выполнения: $( [[ "${EXECUTE}" -eq 1 ]] && echo 'execute' || echo 'dry-run' )"

if [[ "$(pwd)" == /mnt/c/* || "$(pwd)" == /mnt/d/* ]]; then
    echo "Предупреждение: сценарий запущен из каталога на смонтированном диске Windows."
    echo "Рекомендуется хранить ArduPilot внутри файловой системы WSL, например в ${HOME}/src/ardupilot."
fi

if [[ "$ARDUPILOT_ROOT" == /mnt/c/* || "$ARDUPILOT_ROOT" == /mnt/d/* ]]; then
    echo "Предупреждение: целевой каталог ArduPilot расположен на смонтированном диске Windows."
    echo "Рекомендуемый каталог: ${HOME}/src/ardupilot"
fi

mkdir -p "$(dirname "$ARDUPILOT_ROOT")"

if [[ "${EXECUTE}" -ne 1 ]]; then
    echo "Dry-run: автоматические изменения не выполняются."
    echo "Рекомендуемые команды:"
    echo "  mkdir -p \"$(dirname "$ARDUPILOT_ROOT")\""
    echo "  git clone https://github.com/ArduPilot/ardupilot.git \"$ARDUPILOT_ROOT\""
    echo "  cd \"$ARDUPILOT_ROOT\""
    echo "  git submodule update --init --recursive"
    echo "  bash Tools/environment_install/install-prereqs-ubuntu.sh -y"
    echo "  source ~/.profile"
    exit 0
fi

if [[ ! -d "$ARDUPILOT_ROOT/.git" ]]; then
    echo "Клонирование ArduPilot во внешний каталог."
    git clone https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_ROOT"
else
    echo "Каталог ArduPilot уже существует, повторное клонирование не требуется."
fi

cd "$ARDUPILOT_ROOT"

echo "Инициализация подмодулей."
git submodule update --init --recursive

if [[ -f "Tools/environment_install/install-prereqs-ubuntu.sh" ]]; then
    echo "Подготовка зависимостей ArduPilot."
    sudo -v
    bash Tools/environment_install/install-prereqs-ubuntu.sh -y
    echo "После установки зависимостей рекомендуется перезапустить оболочку WSL или выполнить source ~/.profile."
else
    echo "Ошибка: файл Tools/environment_install/install-prereqs-ubuntu.sh не найден."
    exit 1
fi

if [[ -f "Tools/autotest/sim_vehicle.py" ]]; then
    echo "sim_vehicle.py обнаружен: ${ARDUPILOT_ROOT}/Tools/autotest/sim_vehicle.py"
else
    echo "Ошибка: sim_vehicle.py не найден в каталоге ArduPilot."
    exit 1
fi

echo "Отчет сохранен в ${REPORT_PATH}"
