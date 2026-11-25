#!/bin/bash
# fly2.sh X Y Z [timeout]: send goal; follow ONLY this goal's log lines until an outcome
LOG=$HOME/flight_logs/latest/nav.log
~/goal.sh "$1" "$2" "$3" | head -1
GX=$(printf "%.2f" "$1")
timeout 20 bash -c "until grep -q \"Goal received: ($GX\" $LOG; do sleep 1; done"
S=$(grep -n "Goal received: ($GX" $LOG | tail -1 | cut -d: -f1)
timeout "${4:-200}" bash -c "until tail -n +$S $LOG | grep -qE 'DESTINATION REACHED|CRASH:'; do sleep 3; done"
echo "wait rc=$? (124 = timed out)"
tail -n +$S $LOG | sed 's/\x1b\[[0-9;]*m//g' | grep -E 'portfolio\]|start layer|Path found|Replann|Obstacle|DEADLOCK|REACHED|No path|No 2.5D|fallback|failed' | cut -c1-160
~/goal.sh | tail -1
