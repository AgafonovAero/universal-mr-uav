function action_text = print_operator_action(action_title, action_lines)
%PRINT_OPERATOR_ACTION Напечатать требуемое действие оператора.
% Назначение:
%   Формирует человекочитаемый блок сообщения для оператора, когда
%   однокнопочный стенд не может перейти к следующему этапу без ручного
%   действия, административных прав или внешней подготовки среды.
%
% Входы:
%   action_title - краткий заголовок требуемого действия
%   action_lines - строка или строковый массив с шагами оператора
%
% Выходы:
%   action_text - сформированный многострочный текст
%
% Единицы измерения:
%   не применяются
%
% Допущения:
%   Функция используется только для печати пояснений и не влияет на
%   состояние стенда.

action_title = string(action_title);
action_lines = string(action_lines(:));

lines = ["Требуется действие оператора:"; action_title];
for idx = 1:numel(action_lines)
    lines(end + 1, 1) = "  - " + action_lines(idx); %#ok<AGROW>
end

action_text = strjoin(lines, newline);
fprintf('%s\n', char(action_text));
end
