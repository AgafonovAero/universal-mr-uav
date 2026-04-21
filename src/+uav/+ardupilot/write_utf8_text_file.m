function write_utf8_text_file(file_path, text_value)
%WRITE_UTF8_TEXT_FILE Сохранить текст в UTF-8 без BOM.
% Назначение:
%   Создает родительский каталог при необходимости и записывает строковый
%   или символьный текст в указанный файл в кодировке UTF-8 без BOM.
%
% Входы:
%   file_path  - путь к текстовому файлу
%   text_value - текстовое содержимое
%
% Выходы:
%   none
%
% Единицы измерения:
%   не применяются
%
% Допущения:
%   Файл должен быть текстовым артефактом репозитория или временным
%   вспомогательным файлом сценария сопряжения ArduPilot.

file_path = char(string(file_path));
folder_path = fileparts(file_path);
if ~isempty(folder_path) && ~isfolder(folder_path)
    mkdir(folder_path);
end

fid = fopen(file_path, 'w', 'n', 'UTF-8');
if fid < 0
    error('uav:ardupilot:write_utf8_text_file:OpenFailed', ...
        'Не удалось открыть файл %s для записи.', file_path);
end

cleanup_obj = onCleanup(@() fclose(fid)); %#ok<NASGU>
string_value = string(text_value);
string_value(ismissing(string_value)) = "";
fprintf(fid, '%s', char(join(string_value(:), "")));
end
