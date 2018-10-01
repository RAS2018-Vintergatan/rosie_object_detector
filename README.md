# rosie-motor-controller
A motor controller for the robot Rosie

## How to run
Run the following in different terminals:
**Connecting motors**
```
rosrun phidgets motor _serial:=475406 __name:=motorRight _name:=motorRight
```
```
rosrun phidgets motor _serial:=469515 __name:=motorLeft _name:=motorLeft
```

**Running controller**
```
rosparam load controller_params.yaml
```
```
rosrun rosie\_motor\_controller rosie\_motor\_controller
```

**Sending signal to motor controller**
```
rostopic pub /motor_controller/twist geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```
where _linear: x_ is speed forward in m/s and _angualr: x_ is angualar speed in radians/s
