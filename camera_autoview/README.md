# Camera Autoview

Run with ROS 2:

```bash
source /home/agilex/agilex_ws/install/setup.bash
ros2 launch camera_autoview camera_autoview.launch.py
```

Useful overrides:

```bash
ros2 launch camera_autoview camera_autoview.launch.py fullscreen:=false maximize_window:=true
```

This launcher starts in fullscreen with `fill_mode:=crop`.
Click anywhere on the camera image to close it.
