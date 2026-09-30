#!/usr/bin/env bash
# make_slide_videos.sh — turn the hexmap frame folders into videos/GIFs.
#
# plot_linear_hexmap_frames.m writes one folder of numbered PNGs per slide
# segment under results/hexmap_frames/frames/<segment>/ (f001.png ...).
# This assembles each into an .mp4 (and optionally a .gif) next to them.
#
# Usage:
#   ./make_slide_videos.sh                 # all segments, mp4, 12 fps
#   ./make_slide_videos.sh -r 8            # faster playback
#   ./make_slide_videos.sh -g              # also write .gif
#   ./make_slide_videos.sh -s c3_near      # only segments matching a string
#   ./make_slide_videos.sh -p 1.5          # 1.5s pause between clips (default 0.5)
#   ./make_slide_videos.sh -d results/hexmap_frames_v2
#                                          # work on the v2 (hexmap+traces)
#                                          #   frames instead of the default set
#   ./make_slide_videos.sh -c              # ALSO compile every segment into
#                                          #   results/hexmap_frames/all_slides.mp4
#                                          #   in CLOCKWISE order around c3
#   ./make_slide_videos.sh -s b3_near -o b3 -c
#                                          # b3 session: compile its 6 slides
#                                          #   clockwise around b3 into
#                                          #   all_slides_b3.mp4
#   ./make_slide_videos.sh -d results/bidirectional/hexmap_frames_v2 -o c3_bidir -c
#                                          # bidirectional session, all 18
#                                          #   round trips clockwise around c3
#
# NOTE on the -vf pad: the frames are 947x947, an ODD size, and H.264 with
# yuv420p requires even width/height -- without the pad ffmpeg fails with
# "width not divisible by 2". The filter rounds each dimension up to the
# next even number rather than rescaling, so nothing is resampled.
set -euo pipefail

FPS=12
WANT_GIF=0
WANT_CONCAT=0
FILTER=""
# Pause held between clips in the compilation, seconds. Implemented as a
# freeze on each clip's LAST frame rather than black: the hold doubles as a
# moment to read the end state of the slide before the next one starts.
PAUSE_S=0.5
# Which results folder to work in. plot_linear_hexmap_frames.m writes the
# default; plot_linear_hexmap_frames_v2.m writes results/hexmap_frames_v2.
OUT_REL="results/hexmap_frames"
# Which start pad the -c compilation is built around (c3 or b3).
ORIGIN="c3"

while getopts "r:gcs:p:d:o:h" opt; do
  case "$opt" in
    r) FPS="$OPTARG" ;;
    g) WANT_GIF=1 ;;
    c) WANT_CONCAT=1 ;;
    s) FILTER="$OPTARG" ;;
    p) PAUSE_S="$OPTARG" ;;
    d) OUT_REL="${OPTARG%/}" ;;
    o) ORIGIN="$OPTARG" ;;
    h) sed -n '2,25p' "$0"; exit 0 ;;
    *) echo "see -h" >&2; exit 2 ;;
  esac
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FRAME_ROOT="$HERE/$OUT_REL/frames"

if [ ! -d "$FRAME_ROOT" ]; then
  echo "no frames at $FRAME_ROOT -- run the matching plot_linear_hexmap_frames*.m first" >&2
  exit 1
fi
command -v ffmpeg >/dev/null 2>&1 || { echo "ffmpeg not found (brew install ffmpeg)" >&2; exit 1; }

n=0
for d in "$FRAME_ROOT"/*/; do
  seg="$(basename "$d")"
  [ -n "$FILTER" ] && [[ "$seg" != *"$FILTER"* ]] && continue
  [ -e "$d/f001.png" ] || { echo "  skip $seg (no frames)"; continue; }

  ffmpeg -y -loglevel error -framerate "$FPS" -i "$d/f%03d.png" \
    -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2" -pix_fmt yuv420p -c:v libx264 \
    "$d/$seg.mp4"
  echo "  wrote $seg/$seg.mp4"

  if [ "$WANT_GIF" -eq 1 ]; then
    # Two-pass palette: a plain -i *.png GIF looks badly dithered.
    ffmpeg -y -loglevel error -framerate "$FPS" -i "$d/f%03d.png" \
      -vf "palettegen=stats_mode=diff" "$d/.palette.png"
    ffmpeg -y -loglevel error -framerate "$FPS" -i "$d/f%03d.png" -i "$d/.palette.png" \
      -lavfi "paletteuse=dither=bayer:bayer_scale=3" "$d/$seg.gif"
    rm -f "$d/.palette.png"
    echo "  wrote $seg/$seg.gif"
  fi
  n=$((n + 1))
done

# ---- optional: one compilation of every segment, in order --------------
# ORDER IS CLOCKWISE AROUND c3, not alphabetical. Starting from the pad due
# WEST of c3 and sweeping clockwise, the destination angle measured from c3
# decreases monotonically 180 -> -150 deg, so the compilation walks the
# contact once around the board instead of hopping about:
#
#   near (6 immediate neighbours)   b2 180, b3 120, c4  60, d4   0, d3 -60, c2 -120
#   long (12 outer pads)            a1 180, a2 150, a3 120, b4  90, c5  60, d5  30,
#                                   e5   0, e4 -30, e3 -60, d2 -90, c1 -120, b1 -150
#
# Near first, then long -- short hops before the full-width sweeps.
CW_ORDER="\
c3_near_c3_to_b2 c3_near_c3_to_b3 c3_near_c3_to_c4 \
c3_near_c3_to_d4 c3_near_c3_to_d3 c3_near_c3_to_c2 \
c3_long_c3_to_a1 c3_long_c3_to_a2 c3_long_c3_to_a3 \
c3_long_c3_to_b4 c3_long_c3_to_c5 c3_long_c3_to_d5 \
c3_long_c3_to_e5 c3_long_c3_to_e4 c3_long_c3_to_e3 \
c3_long_c3_to_d2 c3_long_c3_to_c1 c3_long_c3_to_b1"
OUT_NAME="all_slides.mp4"

# Same rule around b3 (the b3 session only has the 6 immediate neighbours):
#   a2 180, a3 120, b4 60, c4 0, c3 -60, b2 -120
if [ "$ORIGIN" = "b3" ]; then
  CW_ORDER="\
b3_near_b3_to_a2 b3_near_b3_to_a3 b3_near_b3_to_b4 \
b3_near_b3_to_c4 b3_near_b3_to_c3 b3_near_b3_to_b2"
  OUT_NAME="all_slides_b3.mp4"
elif [ "$ORIGIN" = "c3_bidir" ]; then
  # Bidirectional session (plot_bidir_hexmap_frames*.m): same clockwise
  # order around c3 as above, near then long, under the c3_bidir tag.
  CW_ORDER=$(echo $CW_ORDER | sed -e 's/c3_near_/c3_bidir_/g' -e 's/c3_long_/c3_bidir_/g')
  OUT_NAME="all_slides_c3_bidir.mp4"
elif [ "$ORIGIN" != "c3" ]; then
  echo "unknown -o $ORIGIN (use c3, b3 or c3_bidir)" >&2; exit 2
fi

if [ "$WANT_CONCAT" -eq 1 ]; then
  LIST="$FRAME_ROOT/.concat_list.txt"
  PAUSE_DIR="$FRAME_ROOT/.padded"
  : > "$LIST"
  missing=0
  use_pause=$(awk -v p="$PAUSE_S" 'BEGIN{print (p+0 > 0) ? 1 : 0}')
  [ "$use_pause" -eq 1 ] && { rm -rf "$PAUSE_DIR"; mkdir -p "$PAUSE_DIR"; }
  for seg in $CW_ORDER; do
    clip="$FRAME_ROOT/$seg/$seg.mp4"
    if [ ! -e "$clip" ]; then
      echo "  MISSING $seg -- not in the compilation" >&2
      missing=$((missing + 1))
      continue
    fi
    if [ "$use_pause" -eq 1 ]; then
      # tpad clones the final frame for PAUSE_S. Re-encoded with the same
      # codec/pix_fmt/fps as the source clips, so the -c copy concat below
      # still works. The per-displacement mp4s are left untouched.
      padded="$PAUSE_DIR/$seg.mp4"
      ffmpeg -y -loglevel error -i "$clip" \
        -vf "tpad=stop_mode=clone:stop_duration=$PAUSE_S" \
        -r "$FPS" -pix_fmt yuv420p -c:v libx264 "$padded"
      clip="$padded"
    fi
    printf "file '%s'\n" "$clip" >> "$LIST"
  done
  OUT="$HERE/$OUT_REL/$OUT_NAME"
  n_clips=$(echo $CW_ORDER | wc -w | tr -d " ")
  # -c copy works because every clip was encoded identically above.
  ffmpeg -y -loglevel error -f concat -safe 0 -i "$LIST" -c copy "$OUT"
  rm -f "$LIST"; rm -rf "$PAUSE_DIR"
  echo "  wrote $(basename "$OUT") ($((n_clips - missing))/$n_clips clips, clockwise order around $ORIGIN, ${PAUSE_S}s pause between)"
fi

echo "done: $n segment(s) at ${FPS} fps"
