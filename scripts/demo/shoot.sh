#!/bin/bash
# shoot.sh VIEWS_TSV OUTDIR : one Gazebo (server + full-window 3D view) per world, screenshot each view
set -u
TSV=$1; OUT=$2; mkdir -p "$OUT"
XA=$(ls /run/user/1000/.mutter-Xwaylandauth.* | head -1)
export DISPLAY=:0 WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/run/user/1000 XAUTHORITY=$XA QT_QPA_PLATFORM=xcb
export GZ_SIM_RESOURCE_PATH=$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds
H='<gz-gui><property key="state" type="string">floating</property><property key="showTitleBar" type="bool">false</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="resizable" type="bool">false</property></gz-gui>'
while IFS=$'\t' read -r NAME SRC VIEWS; do
  SRC=$(eval echo "$SRC")
  FIRST=${VIEWS%%;*}; P=${FIRST%%|*}
  cat > /tmp/shot_gui.config <<CFG
<?xml version="1.0"?>
<window><width>1600</width><height>1000</height><menus><drawer visible="false"/><plugins visible="false"/></menus><dialog_on_exit>false</dialog_on_exit></window>
<plugin filename="MinimalScene" name="3D View"><gz-gui><title>3D View</title><property type="bool" key="showTitleBar">false</property><property type="string" key="state">docked</property></gz-gui>
<engine>ogre2</engine><scene>scene</scene><ambient_light>0.55 0.55 0.55</ambient_light><background_color>0.86 0.89 0.93</background_color><camera_pose>$P</camera_pose></plugin>
<plugin filename="GzSceneManager" name="Scene Manager">$H</plugin>
<plugin filename="InteractiveViewControl" name="View control">$H</plugin>
<plugin filename="CameraTracking" name="Camera tracking">$H</plugin>
<plugin filename="VisualizationCapabilities" name="Visualization">$H</plugin>
<plugin filename="Screenshot" name="Screenshot">$H</plugin>
CFG
  gz sim -s -r "$SRC" > /tmp/shot_srv.log 2>&1 &
  sleep 4
  gz sim -g --gui-config /tmp/shot_gui.config > /tmp/shot_gui.log 2>&1 &
  timeout 90 bash -c 'until gz service -l 2>/dev/null | grep -q /gui/screenshot; do sleep 2; done' || echo "$NAME: no GUI"
  sleep 20
  [ "$NAME" = glass_test ] && gz service -s /gui/view/collisions --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 --req 'data: "glass_pane"' >/dev/null
  i=0
  IFS=';' read -ra VS <<< "$VIEWS"
  for v in "${VS[@]}"; do
    i=$((i+1)); p=(${v%%|*}); q=(${v##*|})
    gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 3000 \
      --req "pose: {position: {x: ${p[0]}, y: ${p[1]}, z: ${p[2]}}, orientation: {w: ${q[0]}, x: ${q[1]}, y: ${q[2]}, z: ${q[3]}}}" >/dev/null
    sleep 6
    rm -rf /tmp/shotdir; mkdir -p /tmp/shotdir
    gz service -s /gui/screenshot --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 15000 --req 'data: "/tmp/shotdir"' >/dev/null
    sleep 3
    f=$(ls /tmp/shotdir/*.png 2>/dev/null | head -1); [ -n "$f" ] && mv "$f" "$OUT/${NAME}_$i.png" && echo "$NAME view $i ok" || echo "$NAME view $i FAILED"
  done
  pkill -f "gz sim -g"; pkill -f "gz sim -s"; sleep 4
done < "$TSV"
echo SHOTS_DONE
