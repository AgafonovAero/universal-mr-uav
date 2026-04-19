#!/usr/bin/env bash
set -euo pipefail

ARDUPILOT_ROOT="${ARDUPILOT_ROOT:-$HOME/src/ardupilot}"
REPORT_PATH="${HOME}/ardupilot_setup_report.txt"

mkdir -p "$(dirname "$REPORT_PATH")"
exec > >(tee "$REPORT_PATH") 2>&1

echo "Подготовка внешнего каталога ArduPilot внутри WSL"
echo "  целевой каталог: ${ARDUPILOT_ROOT}"

if [[ "$(pwd)" == /mnt/c/* || "$(pwd)" == /mnt/d/* ]]; then
    echo "Предупреждение: сценарий запущен из каталога, расположенного на смонтированном диске Windows."
    echo "Рекомендуется хранить ArduPilot внутри файловой системы WSL, например в ${HOME}/src/ardupilot."
fi

if [[ "$ARDUPILOT_ROOT" == /mnt/c/* || "$ARDUPILOT_ROOT" == /mnt/d/* ]]; then
    echo "Предупреждение: целевой каталог ArduPilot расположен на смонтированном диске Windows."
    echo "Рекомендуемый каталог: ${HOME}/src/ardupilot"
fi

mkdir -p "$(dirname "$ARDUPILOT_ROOT")"

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
