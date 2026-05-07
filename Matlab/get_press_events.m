function events = get_press_events(df, cell_cols)

events = [];
in_press = false;
rows = [];

for i = 1:height(df)

    if df.ur5_pressing(i) == 1
        if ~in_press
            in_press = true;
            rows = [];
        end
        rows = [rows; i];

    else
        if in_press && ~isempty(rows)

            data = table2array(df(rows, cell_cols));
            peak = max(data,[],1);

            ev.peak = peak;
            ev.peak_max = max(peak);

            events = [events; ev];
        end

        in_press = false;
        rows = [];
    end
end

end