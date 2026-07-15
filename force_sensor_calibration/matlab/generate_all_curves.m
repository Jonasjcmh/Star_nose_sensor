%% generate_all_curves.m
%
% Runs every calibration/compensation analysis script built up in this
% project, in a sensible order, so every curve and coefficient can be
% regenerated with a single command.
%
% Each script stays self-contained (recomputes what it needs from the raw
% logs, per this project's convention) rather than sharing state, so this
% is a thin orchestrator: it just runs each script in turn.
%
% IMPORTANT implementation note: every sub-script begins with its own
% "clear; clc; close all;", which -- because MATLAB's run() executes a
% script in the CALLING workspace -- would also wipe any loop variables
% this orchestrator tried to keep between calls (e.g. a cell array of
% script names plus a loop index). So this file deliberately does NOT use
% a for-loop over a variable; each call recomputes its own path directly
% via mfilename('fullpath'), which reflects the currently executing FILE
% (this orchestrator), not a variable, so it survives each sub-script's
% "clear" untouched.
%
% Order (later scripts don't depend on earlier ones' output, but this
% groups "load cell + UR together" first, then "UR alone", then the
% cross-check that ties them together):
%   1. a1_fit_lc_ur_calibration.m
%        Step 1: load-cell voltage <-> force (ai0 -> F_signed)
%        Step 2/3: F_lc vs UR fz, same-session pairing + per-direction
%                  compensation + Bland-Altman
%        Step 4: ur_only cross-check vs known weight
%   2. plot_lc_vs_ur_by_weight.m
%        LC vs UR force per weight, grouped bars (futek_direct)
%   3. plot_ur_only_vs_load.m
%        UR sensor (fz, absolute) vs known load (ur_only)
%   4. plot_fz_vs_time_ur_only.m
%        UR fz vs time, loaded window only, per weight/direction (ur_only)
%   5. plot_lc_ur_force_vs_time.m
%        Force vs time (-200..+200 g ordered), LC linearization (all raw
%        samples), UR compensation vs F_lc (raw + sign-corrected), UR vs
%        known weight F_true (raw + sign-corrected, UR-side hardware),
%        consolidated coefficients summary
%   6. plot_ur_only_compensation_crosscheck.m
%        Held-out validation: coefficients fit on futek_direct, applied
%        to the independent ur_only dataset
%
% Run this file directly (F5, or "run generate_all_curves" from the
% force_sensor_calibration/matlab folder). No toolboxes required.

clear; clc; close all;

% Plain tic (no output variable) uses MATLAB's single persistent timer,
% which survives the "clear" at the top of every sub-script below --
% unlike a variable such as t_start = tic, which would be wiped by the
% first sub-script's own clear and error out at the final toc.
tic;

fprintf('\n%s\n# Running a1_fit_lc_ur_calibration.m\n%s\n', repmat('#', 1, 78), repmat('#', 1, 78));
run(fullfile(fileparts(mfilename('fullpath')), 'a1_fit_lc_ur_calibration.m'));

fprintf('\n%s\n# Running plot_lc_vs_ur_by_weight.m\n%s\n', repmat('#', 1, 78), repmat('#', 1, 78));
run(fullfile(fileparts(mfilename('fullpath')), 'plot_lc_vs_ur_by_weight.m'));

fprintf('\n%s\n# Running plot_ur_only_vs_load.m\n%s\n', repmat('#', 1, 78), repmat('#', 1, 78));
run(fullfile(fileparts(mfilename('fullpath')), 'plot_ur_only_vs_load.m'));

fprintf('\n%s\n# Running plot_fz_vs_time_ur_only.m\n%s\n', repmat('#', 1, 78), repmat('#', 1, 78));
run(fullfile(fileparts(mfilename('fullpath')), 'plot_fz_vs_time_ur_only.m'));

fprintf('\n%s\n# Running plot_lc_ur_force_vs_time.m\n%s\n', repmat('#', 1, 78), repmat('#', 1, 78));
run(fullfile(fileparts(mfilename('fullpath')), 'plot_lc_ur_force_vs_time.m'));

fprintf('\n%s\n# Running plot_ur_only_compensation_crosscheck.m\n%s\n', repmat('#', 1, 78), repmat('#', 1, 78));
run(fullfile(fileparts(mfilename('fullpath')), 'plot_ur_only_compensation_crosscheck.m'));

elapsed_s = toc;
fprintf('\n%s\n# Done -- 6 scripts, %.1fs\n%s\n', repmat('#', 1, 78), elapsed_s, repmat('#', 1, 78));
