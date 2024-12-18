## Docker

With a SITL running through MissionPlanner, open a Mavlink server port (`Ctrl+F` then `Mavlink`) on TCP port 14550 and then run
```
docker run -it macaw ros2 launch macaw macaw.launch.xml connect_str:="tcp:192.168.0.195:14550"
```
replacing the 192.168.0.195 with your local IP address.

(It ought to work in reverse by exposing the relevant port from the container and then connecting as client from MissionPlanner - but have yet to get that working.)