# To run this framework

```
ros2 launch catvehicle gazebo.launch.py
```
Wait for Gazebo to launch, and the catvehicle is spawned

```
 ros2 launch catvehicle nodes.launch.py 'path:=<insert csv path here>'
```
Wait until theres two logs of "Preprocessing Done" as:

```
[status_pub.py-2] [INFO] [1721499090.362077080] [status_pub]: Preprocessing done
[spawner.py-3] [INFO] [1721499095.011422893] [spawner_despawner]: Preprocessing done
```

Finally...

```
ros2 run catvehicle --ros-args -p 'path:=<insert csv path here, should be same as above>'
```
