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

## Trajectory runs (`trajectory_displacement.py`)

Both 2026-09-30, 1 mm/s, 4 mm indentation, 5 passes out and back, 1 s hold at
every pad, tip engaged for the whole session. Complete.

- `hollow_dome_5_iterations_external_diameter_session_20260930_191043.csv` —
  outer ring a1>a2>a3>b4>c5>d5>e5>e4>e3>d2>c1>b1>a1 (pass ~223 s). Hottest cell
  = pad under the tip in 100/120 holds (83%). Weak spots: e3 (loses to e4 in
  8/10 holds), a3 on the way back (loses to the inner neighbour b3 in 5/5),
  b4 on the way back (to c5/c4, 4/5), d2 back (to d3, 2/5).
- `hollow_dome_5_iterations_internal_diameter_session_20260930_193505.csv` —
  inner ring b2>b3>c4>d4>d3>c2>b2 around c3 (pass ~112 s). 53/60 holds (88%).
  Misses: b3 back (to b4, 3/5), c2 out (to c3, 2/5), d3 back (to d4, 2/5).
- Every miss goes to an ADJACENT cell, usually by a small margin (0.01-0.08
  dC/C0; a3-back is the exception at 0.61 vs 0.43). On the outer ring 9/20
  misses go to the inward neighbour (toward c3) and 7/20 trail (the pad just
  left); e3 loses to e4 in both directions, so e3 looks like a weak cell
  rather than a lag effect.
