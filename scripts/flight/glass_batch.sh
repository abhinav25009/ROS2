#!/bin/bash
# glass_batch.sh SPECFILE OUT: one flight per line "LABEL ENV GX GY"; summary line each in OUT
SPEC=$1; OUT=$2
while read -r LABEL ENVS GX GY; do
  [ -z "$LABEL" ] && continue
  ENVS=${ENVS//,/ }
  ~/glass_flight.sh "$LABEL" "$ENVS" "$GX" "$GY" 1.8 360 > ~/gf_$LABEL.out 2>&1
  L=$(readlink -f ~/flight_logs/latest)
  echo "$LABEL $ENVS goal($GX,$GY) log=$L" >> "$OUT"
done < "$SPEC"
tmux kill-session -t flight 2>/dev/null
echo BATCH_DONE >> "$OUT"
