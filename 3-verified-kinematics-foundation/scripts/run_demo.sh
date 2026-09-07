#!/usr/bin/env bash
# Run the R_sine RViz demo from the VS Code (snap) terminal.
# VS Code is a snap; its terminal injects snap GUI/locale paths that make RViz
# and Gazebo load an incompatible snap libpthread (glibc mismatch crash).
# We scrub those vars, keep the ROS environment, then launch.
set -e
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
for v in LOCPATH GTK_PATH GTK_EXE_PREFIX GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR \
         GTK_IM_MODULE_FILE GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR VSCODE_NLS_CONFIG; do
  unset "$v"
done
export XDG_DATA_DIRS=/usr/share:/usr/local/share
echo "Launching R_sine RViz demo (snap env scrubbed)..."
exec ros2 launch Rsine rsine_sine.launch.py "$@"
