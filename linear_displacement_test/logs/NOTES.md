# linear_displacement_test — log notes

Status notes for individual capture sessions in this folder.

## Failed runs

- `hollow_dome_5_iterations_b3_session_20260918_181142.csv` — **FAILED** (2026-09-18). Run did not complete successfully; data should not be used for analysis.
  Diagnosis (2026-09-30): the log itself is complete (6 neighbours x 5 passes),
  but the robot landed ~3 mm off the b3 pad toward b4 (TCP start -32.8,-497.2 mm
  vs -31.9,-494.5 mm reached as "b3" in the c3 session) and pressed ~0.35 mm
  deeper (Fz ~ -15 N vs -12 N). At slide start the hottest cell is mostly b4,
  and b3->a2 / b3->a3 never reach their destination cell. It is plotted (tag
  `b3_near`) with this caveat -- treat its start/end labels as nominal.

## Bidirectional runs

- `hollow_dome_5_iterations_c3_bidirectional_allpoints_session_20260930_160646.csv`
  (2026-09-30, `bidirectional_displacement.py`): c3 <-> all 18 pads, 5 round
  trips each, 1 mm/s, 4 mm indentation. Complete. Notes from the analysis:
  - c3 <-> c2 never hands off: c2 already sits at ~0.4 dC/C0 while pressing c3
    and c3 stays the hottest cell even at c2 (TCP does reach the c2 position).
    c3 <-> d4 is similarly ambiguous (c3 and d4 equal at the end hold).
  - `fz` reads ~ -130 N throughout (F/T sensor not zeroed); `load_cell_N` is fine.
