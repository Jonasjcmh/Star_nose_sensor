function F = ai0_to_force_N(ai0)
% AI0_TO_FORCE_N  FUTEK 10 lb compression load cell voltage-to-force
% conversion -- the deployed calibration documented in
% force_sensor_calibration/FORCE_CALIBRATION_SOP.md ("its own factory
% voltage-to-force calibration") and implemented as ai0_to_newtons() in
% Integration_2/analyze_session.py:
%
%   F = -(ai0 - AI0_ZERO_V) * (LOADCELL_MAX_N / LOADCELL_V_RANGE)
%
% 5.0 V = 0 N (compression drives ai0 below 5.0 V); rated 10 lb = 44.4822 N
% over a 5 V range -> 8.89644 N/V. This is NOT the same calibration as
% calib_fz_lc_pattern*.json (that one corrects the UR5 wrist fz sensor
% against this FUTEK reading -- unrelated to the ai0->N conversion itself).
    AI0_ZERO_V       = 5.0;
    LOADCELL_MAX_N   = 10.0 * 4.44822;
    LOADCELL_V_RANGE = 5.0;
    LOADCELL_N_PER_V = LOADCELL_MAX_N / LOADCELL_V_RANGE;
    F = -(ai0 - AI0_ZERO_V) * LOADCELL_N_PER_V;
end
