#!/usr/bin/env bash
# Runs the EE tests of ps2autotests (https://github.com/unknownbrackets/ps2autotests,
# ISC licence) through the recompiler and compares what each prints with the output
# recorded on a real PS2. The suite is fetched, not vendored; everything lands in
# build/autotests (gitignored).
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

usage() {
  cat <<'EOF'
usage: scripts/autotests.sh [options] [filter...]

Translates each test ELF with eerecomp, links it against the runtime with a
headless runner that sends the test's stdout (fileio writes to fd 1/2) to a
file, runs it and diffs that with the .expected file. A filter is a substring of
the test path, e.g. cpu/ee_fpu or vu/. With no filter every EE test runs.

options:
  --src DIR      use this ps2autotests checkout instead of fetching one
  --work DIR     output folder (default build/autotests)
  --jobs N       tests built in parallel (default: half the CPUs)
  --timeout S    seconds per test run (default 60)
  -h, --help
Results: <work>/results/<test>.{out,err,diff}, summary in <work>/summary.txt.
EOF
}

REPO_URL=https://github.com/unknownbrackets/ps2autotests
REPO_REV=97469ffbed8631277b94e28d01dabd702aa97ef3
work="$ROOT/build/autotests"; src=""; jobs=$(( $(sysctl -n hw.ncpu 2>/dev/null || nproc) / 2 + 1 )); limit=60; filters=()
while [ $# -gt 0 ]; do
  case "$1" in
    --src) src="$2"; shift ;;
    --work) work="$2"; shift ;;
    --jobs) jobs="$2"; shift ;;
    --timeout) limit="$2"; shift ;;
    -h|--help) usage; exit 0 ;;
    -*) echo "unknown option $1" >&2; exit 2 ;;
    *) filters+=("$1") ;;
  esac
  shift
done
mkdir -p "$work"
work="$(cd "$work" && pwd)"
export SDL_AUDIODRIVER=dummy

if [ -z "$src" ]; then
  src="$work/src"
  if [ ! -d "$src/.git" ]; then
    echo "== fetching ps2autotests"
    git clone -q "$REPO_URL" "$src" || exit 1
  fi
  git -C "$src" checkout -q "$REPO_REV" || exit 1
fi

cxx=clang++; command -v clang++ > /dev/null || cxx=g++
tools="$ROOT/build/tools"
if [ ! -x "$tools/eerecomp" ]; then
  cmake -S "$ROOT" -B "$tools" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=$cxx > /dev/null || exit 1
  ninja -C "$tools" eerecomp > /dev/null || exit 1
fi

echo "== runtime"
# The software rasteriser is enough here: point the wgpu path somewhere empty.
cmake -S "$ROOT" -B "$work/runtime" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=$cxx \
  -DPS2_WGPU_DIR="$work/no-wgpu" > /dev/null || exit 1
ninja -C "$work/runtime" ps2runtime > "$work/runtime/ninja.log" 2>&1 || { tail -30 "$work/runtime/ninja.log"; exit 1; }
flags=(-std=c++20 -O1 -w -I"$ROOT/runtime/include" -I"$ROOT/runtime/src")
$cxx "${flags[@]}" -c "$ROOT/tests/autotests/runner.cpp" -o "$work/runtime/runner.o" || exit 1

tests=()
while IFS= read -r elf; do
  t=${elf#$src/tests/}; t=${t%.elf}
  [ -f "$src/tests/$t.expected" ] || continue
  if [ ${#filters[@]} -gt 0 ]; then
    hit=0; for f in "${filters[@]}"; do [[ "$t" == *"$f"* ]] && hit=1; done
    [ $hit = 1 ] || continue
  fi
  tests+=("$t")
done < <(find "$src/tests" -name '*.elf' | sort)
echo "== ${#tests[@]} tests"

# build_and_run <test>: translate, compile, run; re-translates with the entry points a
# run reports as missing (code only reached through pointers) up to three times.
build_and_run() {
  local t="$1" key="${1//\//_}"
  local gen="$work/gen/$key" obj="$work/obj/$key" res="$work/results/$key" elf="$src/tests/$t.elf"
  mkdir -p "$gen" "$obj" "$work/results"
  : > "$gen.entries"
  local round status="FAIL"
  for round in 1 2 3; do
    rm -f "$gen"/*.cpp "$obj"/*.o "$gen.misses"
    "$tools/eerecomp" "$elf" --entries "$gen.entries" --out "$gen" > "$res.recomp" 2>&1 || { echo "$t RECOMP"; return; }
    local u pids=()
    for u in "$gen"/*.cpp; do
      $cxx "${flags[@]}" -I"$gen" -c "$u" -o "$obj/$(basename "$u" .cpp).o" 2>> "$res.build" & pids+=($!)
    done
    for u in "${pids[@]}"; do wait "$u" || { echo "$t BUILD"; return; }; done
    $cxx -o "$obj/run" "$obj"/*.o "$work/runtime/runner.o" "$work/runtime/libps2runtime.a" -lpthread 2>> "$res.build" || { echo "$t BUILD"; return; }
    perl -e "alarm $limit; exec @ARGV" "$obj/run" "$elf" --misses "$gen.misses" > "$res.out" 2> "$res.err"
    local rc=$?
    if [ -s "$gen.misses" ] && ! /usr/bin/grep -qvxF -f "$gen.entries" "$gen.misses" 2>/dev/null; then :;
    elif [ -s "$gen.misses" ]; then cat "$gen.misses" >> "$gen.entries"; continue; fi
    break
  done
  if diff <(tr -d '\r' < "$src/tests/$t.expected") "$res.out" > "$res.diff"; then status=PASS
  elif [ "${rc:-0}" -ge 128 ]; then status=TIMEOUT
  elif [ ! -s "$res.out" ]; then status=NOOUT
  fi
  local halt; halt=$(sed -n 's/^\[ee\] halted: //p' "$res.err" | head -1)
  printf '%s %s %s\n' "$t" "$status" "$(/usr/bin/grep -c '^[<>]' "$res.diff")${halt:+ halt=${halt// /_}}"
}

rm -f "$work/summary.txt"
running=0
for t in "${tests[@]}"; do
  build_and_run "$t" >> "$work/summary.txt" &
  running=$((running + 1))
  if [ "$running" -ge "$jobs" ]; then wait -n; running=$((running - 1)); fi
done
wait
sort -o "$work/summary.txt" "$work/summary.txt"
column -t "$work/summary.txt"
pass=$(/usr/bin/grep -c ' PASS' "$work/summary.txt")
echo "== $pass/${#tests[@]} pass; diffs in ${work#$ROOT/}/results"
