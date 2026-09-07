#!/usr/bin/env bash
# Launch the thesis drawing demo in RViz from the VS Code (snap) terminal.
# VS Code's snap terminal injects GUI/locale paths that crash RViz (libpthread
# glibc mismatch); we scrub them, keep the ROS environment, then launch.
# Pass launch args through, e.g.:  bash run_thesis_demo.sh surface:=vertical
set -e
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
for v in LOCPATH GTK_PATH GTK_EXE_PREFIX GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR \
         GTK_IM_MODULE_FILE GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR VSCODE_NLS_CONFIG; do
  unset "$v"
done
export XDG_DATA_DIRS=/usr/share:/usr/local/share
echo "Launching thesis drawing demo (snap env scrubbed)..."
exec ros2 launch mycobot_thesis thesis_draw.launch.py "$@"
