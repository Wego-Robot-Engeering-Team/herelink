# herelink_bringup

HereLink RC input is converted to `/cmd_vel` by `herelink_sbus_cmd_vel`,
`limo_ultra_base` subscribes to `/cmd_vel` to drive the robot, and
`camera_autoview` opens the camera in fullscreen by default.

## Run

```bash
source /home/agilex/agilex_ws/install/setup.bash
ros2 launch herelink_bringup herelink_limo_drive.launch.py
```

## Useful launch arguments

```bash
ros2 launch herelink_bringup herelink_limo_drive.launch.py \
  herelink_port:=/dev/ttyACM0 \
  base_port_name:=ttyTHS1 \
  max_linear:=0.5 \
  max_angular:=1.0
```

Disable the camera view if needed:

```bash
ros2 launch herelink_bringup herelink_limo_drive.launch.py enable_camera:=false
```
