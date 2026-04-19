function result = run_wsl_command(command_text, varargin)
%RUN_WSL_COMMAND Выполнить Bash-команду внутри выбранного дистрибутива WSL.
% Назначение:
%   Формирует команду вызова `wsl.exe`, выполняет `bash -lc` внутри
%   выбранного дистрибутива WSL и возвращает структурированный результат.
%   Функция используется как для безопасных проверок среды, так и для
%   операторских действий более высокого уровня.
%
% Входы:
%   command_text - текст Bash-команды
%   varargin     - пары имя-значение:
%                  'DistroName' - имя дистрибутива WSL
%                  'WorkDir'    - рабочий каталог Windows для запуска
%
% Выходы:
%   result - структура с текстом команды, кодом завершения и выводом
%
% Единицы измерения:
%   не применяются
%
% Допущения:
%   Разрешенность действия определяется вызывающим уровнем. Функция только
%   исполняет уже сформированную команду WSL и возвращает ее результат.

options = local_parse_options(varargin{:});
command_text = string(command_text);

powershell_command = "wsl.exe -d " ...
    + local_quote_powershell_literal(options.distro_name) ...
    + " -- bash -lc " ...
    + local_quote_powershell_literal(command_text);
system_text = "powershell.exe -NoProfile -NonInteractive -Command " ...
    + local_quote_for_system(powershell_command);

original_dir = pwd();
cleanup_obj = onCleanup(@() cd(original_dir)); %#ok<NASGU>
cd(char(options.work_dir));
[status_code, output_text] = system(char(system_text));

result = struct();
result.command_text = system_text;
result.status_code = double(status_code);
result.output_text = string(output_text);
result.success = logical(status_code == 0);

if result.success
    result.message = "Команда WSL завершилась успешно.";
else
    result.message = "Команда WSL завершилась с кодом " ...
        + string(status_code) + ".";
end
end

function options = local_parse_options(varargin)
%LOCAL_PARSE_OPTIONS Разобрать пары имя-значение для запуска WSL-команды.

default_cfg = uav.ardupilot.default_json_config();

options = struct();
options.distro_name = string(default_cfg.wsl_distro_name);
options.work_dir = string(pwd());

if mod(numel(varargin), 2) ~= 0
    error( ...
        'uav:setup:run_wsl_command:NameValueCount', ...
        'Ожидались пары имя-значение.');
end

for idx = 1:2:numel(varargin)
    name = string(varargin{idx});
    value = varargin{idx + 1};

    switch lower(name)
        case "distroname"
            options.distro_name = string(value);
        case "workdir"
            options.work_dir = string(value);
        otherwise
            error( ...
                'uav:setup:run_wsl_command:UnknownOption', ...
                'Неизвестный параметр: %s.', ...
                name);
    end
end
end

function quoted = local_quote_for_system(value)
%LOCAL_QUOTE_FOR_SYSTEM Заключить аргумент в двойные кавычки.

value = string(value);
escaped = replace(value, """", """""");
quoted = """" + escaped + """";
end

function quoted = local_quote_powershell_literal(value)
%LOCAL_QUOTE_POWERSHELL_LITERAL Заключить литерал PowerShell в одинарные кавычки.

value = string(value);
escaped = replace(value, "'", "''");
quoted = "'" + escaped + "'";
end
