#!/bin/bash
# record_once.sh SCENE NAME [NAVENV] : run one demo flight and record it (1-min chunks, joined)
SCENE=$1; NAME=$2; NAVENV=${3:-AHPPA_V3=1}
S=/private/tmp/claude-501/-Users-abhinavkumar-uss-projrct/dfd5d4b3-0e8f-47c7-a00e-b30336976a6a/scratchpad
F=/opt/miniconda3/bin/ffmpeg; ST=$S/rec_status.txt
if [ "$SCENE" = glass ]; then
  CAM='pose: {position: {x: -2.1, y: -1.9, z: 2.5}, orientation: {w: 0.9652, x: -0.0224, y: 0.0883, z: 0.2452}}'
else
  CAM='pose: {position: {x: -14.0, y: -13.0, z: 12.0}, orientation: {w: 0.9142, x: -0.0812, y: 0.2286, z: 0.3246}}'
fi
echo "$NAME start $(date +%T)" >> $ST
caffeinate -d -i -t 5400 & CAF=$!
ssh drone-vm "pkill -f 'tail -n [+]1 -f'; tmux kill-session -t demo 2>/dev/null; rm -f ~/demo_glass.out ~/demo_go; tmux new-session -d -s demo 'DEMO_NAVENV=$NAVENV bash ~/demo_flight.sh $SCENE > ~/demo_glass.out 2>&1'"
ssh -o ServerAliveInterval=20 drone-vm 'timeout 900 bash -c "until grep -qE \"DEMO_READY|not ready|no takeoff\" ~/demo_glass.out; do sleep 5; done"; grep -q DEMO_READY ~/demo_glass.out' || { echo "$NAME: bring-up failed" >> $ST; kill $CAF; exit 1; }
ssh drone-vm "gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 --req 'data: \"\"' >/dev/null; gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 3000 --req '$CAM' >/dev/null"
sleep 5
rm -rf $S/rec_$NAME; mkdir -p $S/rec_$NAME
( k=0; while [ ! -f $S/rec_$NAME/stop ]; do k=$((k+1)); screencapture -x -V 60 $S/rec_$NAME/part$(printf %02d $k).mov; done ) & REC=$!
sleep 4
ssh drone-vm 'touch ~/demo_go'
ssh -o ServerAliveInterval=20 drone-vm 'timeout 3600 bash -c "until grep -q DEMO_DONE ~/demo_glass.out; do sleep 3; done"'
sleep 8
touch $S/rec_$NAME/stop; wait $REC
ls $S/rec_$NAME/part*.mov | sed "s/^/file '/; s/$/'/" > $S/rec_$NAME/list.txt
$F -v error -y -f concat -safe 0 -i $S/rec_$NAME/list.txt -c copy $S/rec_$NAME/joined.mov
OUT=$(ssh drone-vm 'L=$(head -1 ~/demo_glass.out | cut -d" " -f2); for p in "DESTINATION REACHED" "CRASH:" IMPACT; do sed "s/\x1b\[[0-9;]*m//g" $L/nav.log | grep -c "$p"; done' | tr '\n' ' ')
echo "$NAME end $(date +%T) reached/crash/impacts: $OUT" >> $S/rec_status.txt
kill $CAF 2>/dev/null
