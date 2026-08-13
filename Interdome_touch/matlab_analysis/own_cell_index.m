function idx = own_cell_index(pt)
% OWN_CELL_INDEX  Column index (1-19, i.e. cell_<idx>) of UR5 point pt's own
% expected capacitive sensor cell. pt is 1-19. Returns NaN if pt is out of
% range or its raw sensor cell isn't one of the 19 used cells.
%
% Mirrors UR5_TO_SENSOR / USED_CELLS in Interdome_touch/analyze_interdome.py
% (must match main.py / Integration_2/ur5_control.py / Integration_2/sensor.py).
    ur5_to_sensor = containers.Map('KeyType', 'double', 'ValueType', 'double');
    raw_by_pt = [24, 12, 0, 37, 25, 13, 1, 50, 38, 26, 14, 2, 51, 39, 27, 15, 52, 40, 28];
    for k = 1:19
        ur5_to_sensor(k) = raw_by_pt(k);
    end

    used_cells = [2, 15, 28, 1, 14, 27, 40, 0, 13, 26, 39, 52, 12, 25, 38, 51, 24, 37, 50];

    if isnan(pt) || ~isKey(ur5_to_sensor, pt)
        idx = NaN;
        return;
    end
    raw = ur5_to_sensor(pt);
    pos = find(used_cells == raw, 1);
    if isempty(pos)
        idx = NaN;
    else
        idx = pos;
    end
end
