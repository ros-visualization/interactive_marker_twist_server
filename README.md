# interactive_marker_twist_server

## Usage
Without specifying a value for the `config` argument, by default, `config` is set to `linear`:
```
ros2 launch interactive_marker_twist_server interactive_markers_launch.py
```
Or
```
ros2 launch interactive_marker_twist_server interactive_markers.launch.xml
```

The `config` argument can be changed (i.e. to `linear`, `planar`, or `aerial`:
```
ros2 launch interactive_marker_twist_server interactive_markers_launch.py config:=planar
```
Or
```
ros2 launch interactive_marker_twist_server interactive_markers.launch.xml config:=planar
```
