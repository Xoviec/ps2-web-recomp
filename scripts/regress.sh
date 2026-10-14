#!/usr/bin/env bash
# Frame-hash regression tests: runs each game headless from save states and
# compares the dumped frames against the hashes in regress/<game>.txt.
# Works with bash 3.2 (stock macOS) and on Linux. See usage below and the
# "Regression tests" section of README.md.
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
here="$ROOT/scripts"

usage() {
  cat <<'EOF'
usage: scripts/regress.sh [check | update | list] [game[:case]]...

  check   run the cases, compare against the stored hashes (default)
  update  run the cases and write their hashes into the manifest
  list    print the cases and whether their state files are present

With no game, every regress/*.txt is used. game:case runs one case.

Manifest regress/<game>.txt, one case per line, # starts a comment:
  <name> <state-file | -> <frames> [runner args...] [<key>=<hash>]...
<frames> counts from the frame the state was saved at (from boot for -).
In runner args a frame number written +N is relative to that frame too, so
`--press +0:cross:300` holds cross for the whole run. <key> names the
renderer and host, e.g. gpu-darwin-arm64 or soft-linux-x86_64, since GPU
output differs between drivers; a case with no hash for this key is MISSING.

Environment:
  REGRESS_STATES   folder holding <game>/<state-file>; default games/<game>/states
                   (all of games/ is gitignored: states hold game RAM, never commit them)
  REGRESS_BIN      runner to test; default build/native-<game>/ps2run
  REGRESS_GS       gpu (default) or soft, passed on as PS2_GS
  REGRESS_JOBS     cases run in parallel, default 1 (~200 MB RSS each)
  REGRESS_TIMEOUT  seconds before a case counts as hung, default 900
  REGRESS_OUT      work folder, default build/regress; failing frames are kept
                   there as PNG next to a diff against the reference frames
EOF
}

mode=check
case "${1:-}" in
  check|update|list) mode=$1; shift ;;
  -h|--help) usage; exit 0 ;;
  -*) usage >&2; exit 2 ;;
esac

gs=${REGRESS_GS:-gpu}
case "$gs" in gpu|soft) ;; *) echo "REGRESS_GS must be gpu or soft" >&2; exit 2 ;; esac
key="$gs-$(uname -s | tr '[:upper:]' '[:lower:]')-$(uname -m)"
jobs_max=${REGRESS_JOBS:-1}
limit=${REGRESS_TIMEOUT:-900}
out_root=${REGRESS_OUT:-$ROOT/build/regress}
# The emulated real-time clock is pinned so a run does not depend on when it
# started; Burnout 3 reads it once while booting.
rtc=1100000000

if command -v md5sum > /dev/null 2>&1; then md5_of() { md5sum "$1" | cut -c1-32; }
else md5_of() { md5 -q "$1"; }
fi
if command -v timeout > /dev/null 2>&1; then with_limit() { timeout "$limit" "$@"; }
else with_limit() { perl -e 'alarm shift; exec @ARGV or exit 127' "$limit" "$@"; }
fi
clock() { perl -MTime::HiRes=time -e 'printf "%.1f\n", time'; }

# Selected games and cases.
games=()
filters=()
if [ $# -eq 0 ]; then
  for m in "$ROOT"/regress/*.txt; do
    [ -f "$m" ] || continue
    g=${m##*/}; games+=("${g%.txt}")
  done
else
  for a in "$@"; do
    g=${a%%:*}
    [ -f "$ROOT/regress/$g.txt" ] || { echo "no regress/$g.txt" >&2; exit 2; }
    seen=0
    for x in ${games[@]+"${games[@]}"}; do [ "$x" = "$g" ] && seen=1; done
    [ $seen = 1 ] || games+=("$g")
    [ "$a" = "$g" ] || filters+=("$a")
  done
fi
[ ${#games[@]} -gt 0 ] || { echo "no regress/*.txt found" >&2; exit 2; }

wanted() { # game case
  [ ${#filters[@]} -eq 0 ] && return 0
  local f
  for f in "${filters[@]}"; do [ "$f" = "$1:$2" ] && return 0; done
  return 1
}

game_elf() {
  local dir="$ROOT/games/$1" boot program
  if [ -f "$dir/core.elf" ]; then echo "$dir/core.elf"; return; fi
  boot=$(sed -n 's/^BOOT2 *= *//p' "$dir/disc/SYSTEM.CNF" 2> /dev/null | tr -d '\r')
  program=${boot#*\\}; program=${program%;*}
  echo "$dir/disc/$program"
}

# Frame the state was saved at, asked of the runner once per state contents.
state_frame() { # bin elf iso state cachedir
  local sum cache n
  sum=$(cksum < "$4" | tr ' ' '_')
  cache="$5/${4##*/}.$sum.frame"
  if [ ! -s "$cache" ]; then
    n=$(SDL_AUDIODRIVER=dummy PS2_GS=soft PS2_RTC=$rtc with_limit "$1" "$2" --disc "$3" --load-state "$4" --frames 1 2>&1 \
          | sed -n 's/^\[state\] loaded frame \([0-9]*\).*/\1/p' | head -1)
    [ -n "$n" ] || return 1
    echo "$n" > "$cache"
  fi
  cat "$cache"
}

# Writes "<status>|<hash>|<seconds>|<detail>" to $dir/result.
run_case() { # game name state frames hashref args...
  local game=$1 name=$2 state=$3 frames=$4 want=$5; shift 5
  local dir="$out/$name" states="$states_dir" t0 start=0 end every d lo hi a status hash detail
  local -a args=()
  rm -rf "$dir"; mkdir -p "$dir/frames"
  t0=$(clock)
  finish() { echo "$1|$2|$(perl -e 'printf "%.1f", $ARGV[1] - $ARGV[0]' "$t0" "$(clock)")|$3" > "$dir/result"; }
  if [ "$state" != - ]; then
    [ -f "$states/$state" ] || { finish MISSING "" "no state $states/$state"; return; }
    start=$(state_frame "$bin" "$elf" "$iso" "$states/$state" "$out_root/.frames") || { finish FAIL "" "cannot load $state"; return; }
    args=(--load-state "$states/$state")
  fi
  end=$((start + frames))
  # Hash a handful of frames: dump every <every> frames, with <every> a divisor
  # of the last frame between frames/16 and frames/4, so the final frame is
  # always among them. Without one, only the final frame is dumped.
  every=$end
  lo=$((frames / 16)); [ $lo -ge 1 ] || lo=1
  hi=$((frames / 4))
  d=$hi
  while [ $d -ge $lo ]; do
    if [ $((end % d)) -eq 0 ]; then every=$d; break; fi
    d=$((d - 1))
  done
  for a in "$@"; do
    case "$a" in +[0-9]*) a="$((start + ${a%%:*}))${a#"${a%%:*}"}" ;; esac
    args+=("$a")
  done
  SDL_AUDIODRIVER=dummy PS2_GS=$gs PS2_RTC=$rtc with_limit "$bin" "$elf" --disc "$iso" --memcard "$dir/memcard.ps2" \
    --frames "$end" --frames-dir "$dir/frames" --frame-every "$every" ${args[@]+"${args[@]}"} > "$dir/run.log" 2>&1
  status=$?
  if ! grep -q "^\[ee\] stopped after $end frames" "$dir/run.log"; then
    detail="runner exit $status"
    [ $status -eq 142 ] || [ $status -eq 124 ] && detail="timed out after ${limit}s"
    finish FAIL "" "$detail, see $dir/run.log"
    return
  fi
  (cd "$dir/frames" && for f in *.ppm; do echo "$f $(md5_of "$f")"; done) > "$dir/frames.txt"
  hash=$(md5_of "$dir/frames.txt")
  if [ "$mode" = update ]; then
    rm -rf "$refs/$name"; mkdir -p "$refs/$name"
    mv "$dir/frames"/*.ppm "$refs/$name/"; cp "$dir/frames.txt" "$refs/$name/"
    finish UPDATED "$hash" "$(wc -l < "$dir/frames.txt" | tr -d ' ') frames up to $end"
  elif [ -z "$want" ]; then
    finish MISSING "$hash" "no $key hash yet (run update)"
  elif [ "$hash" = "$want" ]; then
    rm -rf "$dir/frames"
    finish PASS "$hash" ""
  else
    # Keep the frames for viewing: each one as PNG and, where a reference
    # frame from the last update exists and differs, a diff image beside it.
    local f base pixels first=""
    for f in "$dir/frames"/*.ppm; do
      base=${f##*/}
      if [ -f "$refs/$name/$base" ] && ! cmp -s "$f" "$refs/$name/$base"; then
        pixels=$(python3 -I "$here/regress_img.py" diff "$refs/$name/$base" "$f" "${f%.ppm}.diff.png")
        [ -n "$first" ] || first="first differs at ${base%.ppm} ($pixels pixels)"
      fi
      python3 -I "$here/regress_img.py" png "$f" "${f%.ppm}.png" && rm -f "$f"
    done
    if [ -z "$first" ]; then
      if [ -f "$refs/$name/frames.txt" ]; then first="dumped frames differ from the reference set ($refs/$name)"
      else first="no reference frames in $refs/$name to diff against"; fi
    fi
    detail="$first; frames in $dir/frames"
    finish FAIL "$hash" "$detail"
  fi
}

total_fail=0
for game in "${games[@]}"; do
  manifest="$ROOT/regress/$game.txt"
  if [ -n "${REGRESS_STATES:-}" ]; then states_dir="$REGRESS_STATES/$game"; else states_dir="$ROOT/games/$game/states"; fi
  bin=${REGRESS_BIN:-$ROOT/build/native-$game/ps2run}
  elf=$(game_elf "$game")
  iso="$ROOT/games/$game/disc.iso"
  out="$out_root/$game"
  refs="$states_dir/refs/$key"
  mkdir -p "$out" "$out_root/.frames"

  names=()
  while read -r name state frames rest; do
    case "$name" in ''|'#'*) continue ;; esac
    wanted "$game" "$name" || continue
    names+=("$name")
    if [ "$mode" = list ]; then
      if [ "$state" = - ] || [ -f "$states_dir/$state" ]; then s=ok; else s="missing $states_dir/$state"; fi
      printf '%-10s %-14s %-24s %6s frames  state %s\n' "$game" "$name" "$state" "$frames" "$s"
    fi
  done < "$manifest"
  [ "$mode" = list ] && continue
  [ ${#names[@]} -gt 0 ] || continue
  [ -x "$bin" ] || { echo "$game: no runner at $bin" >&2; total_fail=1; continue; }
  [ -f "$elf" ] || { echo "$game: no executable at $elf" >&2; total_fail=1; continue; }
  [ -e "$iso" ] || { echo "$game: no disc at $iso" >&2; total_fail=1; continue; }

  echo "== $game ($key, $bin)"
  game_t0=$(clock)
  while read -r name state frames rest; do
    case "$name" in ''|'#'*) continue ;; esac
    wanted "$game" "$name" || continue
    want=""; cargs=()
    for tok in $rest; do
      case "$tok" in
        "$key"=*) want=${tok#*=} ;;
        gpu-*=*|soft-*=*) ;;
        *) cargs+=("$tok") ;;
      esac
    done
    while [ "$(jobs -rp | wc -l | tr -d ' ')" -ge "$jobs_max" ]; do sleep 0.2; done
    run_case "$game" "$name" "$state" "$frames" "$want" ${cargs[@]+"${cargs[@]}"} &
  done < "$manifest"
  wait

  fails=0
  for name in "${names[@]}"; do
    IFS='|' read -r status hash secs detail < "$out/$name/result"
    printf '%-8s %-14s %6ss  %s\n' "$status" "$name" "$secs" "${detail:-$hash}"
    [ "$status" = FAIL ] && fails=$((fails + 1))
  done
  printf '%s: %s case(s) in %ss\n' "$game" "${#names[@]}" "$(perl -e 'printf "%.1f", $ARGV[1] - $ARGV[0]' "$game_t0" "$(clock)")"
  [ $fails -eq 0 ] || total_fail=1

  if [ "$mode" = update ]; then
    tmp="$manifest.tmp.$$"
    while IFS= read -r line; do
      set -- $line
      n=${1:-}
      if [ -n "$n" ] && [ "${n#\#}" = "$n" ] && [ -f "$out/$n/result" ]; then
        IFS='|' read -r status hash secs detail < "$out/$n/result"
        if [ "$status" = UPDATED ]; then
          case " $line " in
            *" $key="*) line=$(printf '%s\n' "$line" | sed "s/\([[:space:]]\)$key=[0-9a-f]*/\1$key=$hash/") ;;
            *) line="$line $key=$hash" ;;
          esac
        fi
      fi
      printf '%s\n' "$line"
    done < "$manifest" > "$tmp" && mv "$tmp" "$manifest"
    echo "wrote $key hashes to ${manifest#$ROOT/}; reference frames in $refs"
  fi
done
exit $total_fail
