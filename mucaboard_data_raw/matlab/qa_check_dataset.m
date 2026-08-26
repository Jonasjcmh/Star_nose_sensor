function report = qa_check_dataset(csv_path, varargin)
% QA_CHECK_DATASET  Run before trusting a mucaboard_data_raw session log.
%
%   report = qa_check_dataset(csv_path)
%   report = qa_check_dataset(csv_path, 'N_ITERATIONS', 10)
%
% Runs the checks that would have caught every real problem found so far
% in this project's raw mucaboard sessions:
%   1. Row integrity      -- every row has the expected 59 fields.
%   2. Impossible values  -- any raw cell reading above RAW_VALID_MAX
%                             (real readings never exceed ~70; a 16-bit
%                             overflow lands around 65500-65535).
%   3. Baseline homogeneity -- do the 19 calib_i values cluster together,
%                             or is one pad calibrated wildly differently
%                             from the rest (flags a bad calibration
%                             snapshot or a genuinely different pad)?
%   4. Non-contact drift   -- for each of the 19 raw cells, compare its
%                             mean reading during NON-CONTACT phases
%                             (locate/post) in the FIRST vs LAST third of
%                             the session. A real sensor should return to
%                             ~calib every time nothing is touching it;
%                             a cell that permanently drifts up (like
%                             cell_5/cell_2 in the earlier solid file)
%                             shows up here even though its value alone
%                             isn't "impossible".
%   5. Own-cell localization -- for each of the 19 points, is the
%                             press-locked response (hold minus
%                             locate/post) BIGGEST on that point's own
%                             cell (identity: cell_k = point k), or does
%                             some other point's press light up that
%                             cell more? A ratio <= 1 means the pad isn't
%                             behaving like a localized sensor for that
%                             press -- could be genuine hardware trouble,
%                             not just weak coupling.
%
% Prints a plain-text report and returns a struct with the same numbers,
% including a top-level report.ok (true only if nothing failed).
%
% This deliberately does NOT know about TRUE_TO_CODE / board relabeling
% -- it works entirely in the robot's own ur5_point numbering, since
% that's what's actually in the file, and drift/saturation are hardware
% properties of a raw column, independent of what a human calls it.

    p = inputParser;
    p.addParameter('N_ITERATIONS', 10);
    p.addParameter('RAW_VALID_MAX', 1000);
    p.addParameter('DRIFT_THRESHOLD', 10);    % raw counts
    p.parse(varargin{:});
    N_ITERATIONS    = p.Results.N_ITERATIONS;
    RAW_VALID_MAX   = p.Results.RAW_VALID_MAX;
    DRIFT_THRESHOLD = p.Results.DRIFT_THRESHOLD;

    [~, fname, fext] = fileparts(csv_path);
    fprintf('\n%s\n', repmat('=', 1, 70));
    fprintf('QA CHECK: %s%s\n', fname, fext);
    fprintf('%s\n', repmat('=', 1, 70));

    report = struct('file', csv_path, 'ok', true, 'issues', {{}});

    % ---- 0. Read everything -------------------------------------------
    fid = fopen(csv_path, 'r');
    if fid < 0
        error('qa_check_dataset:noread', 'Could not open %s', csv_path);
    end
    header_line  = fgetl(fid);
    column_names = strsplit(header_line, ',', 'CollapseDelimiters', false);
    n_expected_fields = numel(column_names);

    text_rows  = {};
    n_bad_rows = 0;
    while true
        line = fgetl(fid);
        if ~ischar(line)
            break;
        end
        parts = strsplit(line, ',', 'CollapseDelimiters', false);
        if numel(parts) ~= n_expected_fields
            n_bad_rows = n_bad_rows + 1;
            continue;   % skip -- can't safely index a malformed row
        end
        text_rows{end + 1} = parts; %#ok<AGROW>
    end
    fclose(fid);
    n_rows    = numel(text_rows);
    data_text = vertcat(text_rows{:});

    fprintf('\n[1] ROW INTEGRITY\n');
    fprintf('    %d data rows read, expected %d fields/row\n', n_rows, n_expected_fields);
    if n_bad_rows > 0
        fprintf('    FAIL: %d malformed row(s) skipped (wrong field count)\n', n_bad_rows);
        report.ok = false;
        report.issues{end+1} = sprintf('%d malformed rows', n_bad_rows);
    else
        fprintf('    OK: every row has exactly %d fields\n', n_expected_fields);
    end

    column_index = @(name) find(strcmp(column_names, name));
    get_numbers  = @(name) str2double(data_text(:, column_index(name)));

    ur5_point = get_numbers('ur5_point');
    round_idx = get_numbers('round_idx');
    phase     = data_text(:, column_index('phase'));

    raw_cells = nan(n_rows, 19);
    for k = 1:19
        raw_cells(:, k) = get_numbers(sprintf('cell_%d', k));
    end
    calib_raw = nan(1, 19);
    for k = 1:19
        one_column = get_numbers(sprintf('calib_%d', k));
        calib_raw(k) = one_column(1);
    end

    % ---- 2. Impossible values ------------------------------------------
    fprintf('\n[2] IMPOSSIBLE VALUES (> %g raw counts)\n', RAW_VALID_MAX);
    bad_mask  = raw_cells > RAW_VALID_MAX;
    n_bad_val = sum(bad_mask(:));
    if n_bad_val > 0
        bad_cells = find(any(bad_mask, 1));
        fprintf('    FAIL: %d/%d readings exceed %g -- affects cell(s): %s\n', ...
            n_bad_val, numel(raw_cells), RAW_VALID_MAX, mat2str(bad_cells));
        report.ok = false;
        report.issues{end+1} = sprintf('impossible values in cell(s) %s', mat2str(bad_cells));
    else
        fprintf('    OK: no readings exceed %g\n', RAW_VALID_MAX);
    end
    raw_cells(bad_mask) = NaN;   % don't let them corrupt checks 3-5 below

    % ---- 3. Baseline homogeneity ---------------------------------------
    fprintf('\n[3] BASELINE (calib) HOMOGENEITY\n');
    cmean = mean(calib_raw);
    cstd  = std(calib_raw);
    fprintf('    calib_1..19: mean=%.2f  std=%.2f  range=[%.1f, %.1f]\n', ...
        cmean, cstd, min(calib_raw), max(calib_raw));
    outlier_k = find(abs(calib_raw - cmean) > 3 * cstd);
    if ~isempty(outlier_k)
        fprintf('    FAIL: calib outlier(s) beyond 3sd of the other pads: %s\n', mat2str(outlier_k));
        for k = outlier_k
            fprintf('        calib_%d = %.1f\n', k, calib_raw(k));
        end
        report.ok = false;
        report.issues{end+1} = sprintf('calib outlier(s) %s', mat2str(outlier_k));
    else
        fprintf('    OK: all 19 calib values within 3sd of each other\n');
    end

    % ---- 4. Non-contact drift ------------------------------------------
    fprintf('\n[4] NON-CONTACT DRIFT (locate/post, first third vs last third of session)\n');
    noncontact = strcmp(phase, 'locate') | strcmp(phase, 'post');
    n_noncontact = sum(noncontact);
    idx_noncontact = find(noncontact);
    third = max(1, floor(numel(idx_noncontact) / 3));
    early_idx = idx_noncontact(1:third);
    late_idx  = idx_noncontact(end - third + 1:end);

    drifted = [];
    for k = 1:19
        early_mean = mean(raw_cells(early_idx, k), 'omitnan');
        late_mean  = mean(raw_cells(late_idx,  k), 'omitnan');
        drift = late_mean - early_mean;
        if abs(drift) > DRIFT_THRESHOLD
            drifted(end+1) = k; %#ok<AGROW>
            fprintf('    cell_%-2d  early=%.1f  late=%.1f  drift=%+.1f  <-- FLAG\n', ...
                k, early_mean, late_mean, drift);
        end
    end
    if isempty(drifted)
        fprintf('    OK: no cell drifted more than %g counts between session start and end\n', DRIFT_THRESHOLD);
    else
        fprintf('    FAIL: %d cell(s) drifted more than %g counts while untouched: %s\n', ...
            numel(drifted), DRIFT_THRESHOLD, mat2str(drifted));
        report.ok = false;
        report.issues{end+1} = sprintf('drifted cell(s) %s', mat2str(drifted));
    end

    % ---- 5. Own-cell localization ---------------------------------------
    fprintf('\n[5] OWN-CELL LOCALIZATION (identity mapping: cell_k = point k)\n');
    weak_points = [];
    for code = 1:19
        hold_rows = (ur5_point == code) & strcmp(phase, 'hold') & ...
                    (round_idx >= 0) & (round_idx <= N_ITERATIONS - 1);
        rest_rows = (ur5_point == code) & (strcmp(phase, 'locate') | strcmp(phase, 'post'));
        if ~any(hold_rows) || ~any(rest_rows)
            fprintf('    point %2d: SKIPPED (no hold/rest rows found)\n', code);
            continue;
        end
        own_delta = mean(raw_cells(hold_rows, code), 'omitnan') - mean(raw_cells(rest_rows, code), 'omitnan');

        other_deltas = nan(1, 19);
        for oc = 1:19
            if oc == code, continue; end
            oh = (ur5_point == oc) & strcmp(phase, 'hold') & (round_idx >= 0) & (round_idx <= N_ITERATIONS - 1);
            or_ = (ur5_point == oc) & (strcmp(phase, 'locate') | strcmp(phase, 'post'));
            if ~any(oh) || ~any(or_), continue; end
            other_deltas(oc) = mean(raw_cells(oh, code), 'omitnan') - mean(raw_cells(or_, code), 'omitnan');
        end
        avg_other = mean(other_deltas, 'omitnan');
        ratio = own_delta / avg_other;
        if ~(ratio > 1)
            weak_points(end+1) = code; %#ok<AGROW>
            fprintf('    point %2d: own_delta=%.2f  avg_other=%.2f  ratio=%.2f  <-- FLAG\n', ...
                code, own_delta, avg_other, ratio);
        end
    end
    if isempty(weak_points)
        fprintf('    OK: every point''s own cell responds more than the average cross-talk\n');
    else
        fprintf('    FAIL: %d point(s) do NOT show a localized own-cell response: %s\n', ...
            numel(weak_points), mat2str(weak_points));
        report.ok = false;
        report.issues{end+1} = sprintf('non-localized point(s) %s', mat2str(weak_points));
    end

    % ---- Summary ----------------------------------------------------------
    fprintf('\n%s\n', repmat('-', 1, 70));
    if report.ok
        fprintf('RESULT: PASS -- no issues found, safe to analyze.\n');
    else
        fprintf('RESULT: FAIL -- %d issue(s) found:\n', numel(report.issues));
        for i = 1:numel(report.issues)
            fprintf('    - %s\n', report.issues{i});
        end
    end
    fprintf('%s\n', repmat('=', 1, 70));
end
