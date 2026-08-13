# Interdome_touch — ML approach options (position + depth detection)

Notes from evaluating whether the `interdome_*.csv` dataset (19 capacitive
cells + FUTEK force, logged per point/depth/iteration/phase) is a good fit
for a sequence model, and which architecture to start with.

## Why not jump straight to Mamba

Mamba's advantage is efficient modeling of *long* sequences. Each touch
cycle here (press → hold → retract) is short — the sample-count breakdown
in `analyze_interdome.py` shows press/retract phases have far fewer rows
than hold per point/depth/iteration. That short sequence length means
Mamba's long-context strength isn't needed; a lighter model likely performs
just as well with far less setup (no CUDA SSM kernels, no GPU requirement).

The bigger practical issues are:
- Phase class imbalance (hold vastly outnumbers press/retract)
- Whether depth/position can be read off the hold-phase steady-state alone,
  or actually require the ramp dynamics (press/retract shape)

## 1. Task framing

| Axis | Options | Tradeoff |
|---|---|---|
| Position target | 19 main hex points only, vs. all 85 labels (main + horizontal + diagonal + triangle) | 85-class is the real interpolated-position task but has far fewer samples per class (press/retract counts are already thin); 19-class is easier and matches the physical sensor cells 1:1 |
| Depth target | Classification (5 discrete levels: 0–4mm) vs. regression (continuous mm) | Classification is simpler and matches how the data was collected (discrete depth steps); regression could generalize between steps but there's no continuous ground truth to justify it here |
| Input window | Whole press→hold→retract cycle, vs. hold-phase snapshot only | Whole cycle captures ramp dynamics (rise time correlates with depth) but is a much longer/noisier sequence; hold-only is a static feature vector, throws away timing info |

## 2. Model family

(Assuming sequence framing — whole press→hold→retract cycle → one
(position, depth) label.)

| Model | Why consider it | Why maybe not |
|---|---|---|
| **Aggregate features + gradient-boosted trees (LightGBM/XGBoost) or MLP** | Fastest to build, strong baseline, interpretable (feature importance tells you which cells matter), no sequence modeling needed if hold-phase alone is separable | Discards ramp-timing info; won't help if depth is only distinguishable from the *rate* of press, not the plateau |
| **1D temporal CNN** | Cheap, captures local ramp shape, easy to train on short sequences, good middle ground | Limited receptive field unless stacked; slightly more setup than trees |
| **GRU/LSTM** | Naturally models the press→hold→retract sequence order, handles variable-length windows | Slower to train than CNN, more hyperparameter sensitivity, still probably overkill for how short each cycle is |
| **Mamba/SSM** | Only worth it if sequences turn out to be long (e.g. keeping full multi-iteration context) or you want SOTA long-range modeling | Heaviest to set up correctly (needs the mamba-ssm CUDA kernels, GPU), and sequence lengths here don't need its long-context strength |

## Recommendation

1. Start with aggregate-features + gradient-boosted trees as a first
   baseline — cheap, and tells you quickly whether hold-phase alone is
   enough to separate 19 or 85 positions × 5 depths.
2. If depth accuracy is weak, move to a 1D-CNN over the full ramp
   (press→hold→retract) to see if ramp dynamics help.
3. Skip Mamba unless the CNN/GRU baseline shows a need for much longer
   context than a single touch cycle.
