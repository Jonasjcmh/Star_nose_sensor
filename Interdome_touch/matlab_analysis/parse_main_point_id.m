function id = parse_main_point_id(label)
% PARSE_MAIN_POINT_ID  Numeric UR5 point id (1-19) from a 'point' column
% label, e.g. 'P01' -> 1, 'P19' -> 19, or a plain '14' -> 14 (older logs
% store bare integers). Returns NaN if no digits are present.
%
% Uses digit-extraction rather than regexp: this project's MATLAB code
% avoids regexp for filename/label parsing (see project memory
% matlab-script-verification) because MATLAB's regexp silently fails to
% match optional named groups that Octave/Python accept.
    digits = label(isstrprop(label, 'digit'));
    if isempty(digits)
        id = NaN;
    else
        id = str2double(digits);
    end
end
