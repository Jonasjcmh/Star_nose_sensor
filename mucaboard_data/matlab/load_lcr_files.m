function d = load_lcr_files(paths)
% LOAD_LCR_FILES  Load and concatenate multiple LCR-connected capacitance
% CSVs (read_lcr_csv.m) into one combined struct -- used because flat/solid
% each have their LCR sweep split across 5 files (one per depth).
    d.point = [];
    d.depth_mm = [];
    d.phase = {};
    d.load_cell_N = [];
    d.Cp_pF = [];
    for i = 1:numel(paths)
        di = read_lcr_csv(paths{i});
        d.point       = [d.point; di.point];
        d.depth_mm    = [d.depth_mm; di.depth_mm];
        d.phase       = [d.phase; di.phase];
        d.load_cell_N = [d.load_cell_N; di.load_cell_N];
        d.Cp_pF       = [d.Cp_pF; di.Cp_pF];
    end
    d.n_rows = numel(d.point);
end
