function result = run_powershell_script(script_path, argument_list, varargin)
%RUN_POWERSHELL_SCRIPT Выполнить PowerShell-сценарий с журналированием.
% Назначение:
%   Формирует команду запуска PowerShell-сценария, при необходимости
%   добавляет параметр Execute, выполняет сценарий и возвращает
%   структурированный результат. При задании пути к журналу сохраняет
%   выходной текст в UTF-8 без BOM.
%
% Входы:
%   script_path    - путь к PowerShell-сценарию
%   argument_list  - строковый массив аргументов сценария
%   varargin       - пары:
%                    'ExecuteFlag' - логический флаг добавления -Execute
%                    'LogPath'     - путь к файлу журнала
%                    'WorkDir'     - рабочий каталог команды
%
% Выходы:
%   result - структура с текстом команды, кодом завершения и выводом
%
% Единицы измерения:
%   не применяются
%
% Допущения:
%   Сценарий PowerShell сам определяет, какие действия безопасны в режиме
%   проверки, а какие требуют явного параметра -Execute.

options = local_parse_options(varargin{:});

script_path = string(script_path);
if nargin < 2 || isempty(argument_list)
    argument_list = strings(0, 1);
else
    argument_list = string(argument_list(:));
end

if ~isfile(char(script_path))
    result = local_make_result("", 1, "", false, ...
        "PowerShell-сценарий не найден: " + script_path);
    return;
end

if options.execute_flag && ~any(strcmpi(argument_list, "-Execute"))
    argument_list(end + 1, 1) = "-Execute";
end

command_text = "powershell -ExecutionPolicy Bypass -File " ...
    + local_quote_argument(script_path);
for idx = 1:numel(argument_list)
    command_text = command_text + " " + local_quote_argument(argument_list(idx));
end

original_dir = pwd();
cleanup_obj = onCleanup(@() cd(original_dir)); %#ok<NASGU>
cd(char(options.work_dir));
[status_code, output_text] = system(char(command_text));
output_text = string(output_text);

if strlength(options.log_path) > 0
    local_write_utf8_text(options.log_path, output_text);
end

result = local_make_result( ...
    command_text, ...
    status_code, ...
    output_text, ...
    status_code == 0, ...
    local_status_message(status_code));
end

function options = local_parse_options(varargin)
%LOCAL_PARSE_OPTIONS Разобрать параметры функции.

options = struct();
options.execute_flag = false;
options.log_path = "";
options.work_dir = string(pwd());

if mod(numel(varargin), 2) ~= 0
    error( ...
        'uav:setup:run_powershell_script:NameValueCount', ...
        'Ожидались пары имя-значение.');
end

for idx = 1:2:numel(varargin)
    name = string(varargin{idx});
    value = varargin{idx + 1};
    switch lower(name)
        case "executeflag"
            options.execute_flag = logical(value);
        case "logpath"
            options.log_path = string(value);
        case "workdir"
            options.work_dir = string(value);
        otherwise
            error( ...
                'uav:setup:run_powershell_script:UnknownOption', ...
                'Неизвестный параметр: %s.', ...
                name);
    end
end
end

function result = local_make_result(command_text, status_code, output_text, success, message)
%LOCAL_MAKE_RESULT Сформировать структуру результата запуска.

result = struct();
result.command_text = string(command_text);
result.status_code = double(status_code);
result.output_text = string(output_text);
result.success = logical(success);
result.message = string(message);
end

function text_value = local_status_message(status_code)
%LOCAL_STATUS_MESSAGE Сформировать пояснение по коду завершения.

if status_code == 0
    text_value = "PowerShell-сценарий завершился успешно.";
else
    text_value = "PowerShell-сценарий завершился с кодом " + string(status_code) + ".";
end
end

function quoted = local_quote_argument(value)
%LOCAL_QUOTE_ARGUMENT Заключить аргумент команды в кавычки при необходимости.

value = string(value);
escaped = replace(value, """", "\""");
if strlength(escaped) == 0 || contains(escaped, " ") || contains(escaped, ":") ...
        || contains(escaped, "\") || contains(escaped, "/")
    quoted = """" + escaped + """";
else
    quoted = escaped;
end
end

function local_write_utf8_text(path_value, text_value)
%LOCAL_WRITE_UTF8_TEXT Сохранить текст в UTF-8 без BOM.

path_value = string(path_value);
folder_path = fileparts(char(path_value));
if strlength(string(folder_path)) > 0 && ~isfolder(folder_path)
    mkdir(folder_path);
end

fid = fopen(char(path_value), 'w', 'n', 'UTF-8');
if fid < 0
    error( ...
        'uav:setup:run_powershell_script:LogOpen', ...
        'Не удалось открыть журнал %s.', ...
        path_value);
end

cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', char(text_value));
end
