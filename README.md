# simple_sim_ros

Minimally featured but fast ROS physics simulation wrapping bullet

```
roslaunch bullet_server bullet_server.launch
roslaunch bullet_server stewart_platform.launch
```

The ros pid package is required, for example `sudo apt-get install ros-kinetic-pid` on kinetic systems.

Then extend on 'actuator':

Command a position to an actuator (prismatic_0 through prismatic_6), this will
cause actuator 3 to retract:

```
rostopic pub /prismatic_3/setpoint std_msgs/Float64 "data: 0.1"  -1
```

Give periodic impulses to an actuator (rough motion):

```
rostopic pub /add_impulse bullet_server/Impulse "{body: 'top_cylinder_0', impulse:{x: -0.1}}" -r 4
```

Demonstration video:

https://www.youtube.com/watch?v=URoyLptHvCU

# old instructions (now out of date)

```
roslaunch bullet_server bullet_server.launch
rosrun bullet_server random_body.py
rosrun bullet_server test.py  # drop a small 'car'
```

Ubuntu 16.04 + Kinetic

```
rosrun bullet_server mesh.py _file_name:=`rospack find bullet_server`/data/dodecasphere.stl
Traceback (most recent call last):
  File "/home/lucasw/catkin_ws/src/simple_sim_ros/bullet_server/scripts/mesh.py", line 18, in <module>
    from pyassimp import pyassimp
  File "/usr/lib/python2.7/dist-packages/pyassimp/__init__.py", line 1, in <module>
    from .core import *
  File "/usr/lib/python2.7/dist-packages/pyassimp/core.py", line 29, in <module>
    class AssimpLib(object):
  File "/usr/lib/python2.7/dist-packages/pyassimp/core.py", line 33, in AssimpLib
    load, load_mem, release, dll = helper.searc
```
pyassimp api differences from 14.04?

# Dependencies

```
sudo apt-get install python-pyassimp
```

# misc

Generic mesh loader for python?

assimp

vtk stl reader?
No don't like it.

## SoftBody

MeshPy for generating triangle and tetras?

## Debug

```
rviz: /build/ogre-1.9-mqY1wq/ogre-1.9-1.9.0+dfsg1/OgreMain/include/OgreAxisAlignedBox.h:252: void Ogre::AxisAlignedBox::setExtents(const Ogre::Vector3&, const Ogre::Vector3&): Assertion `(min.x <= max.x && min.y <= max.y && min.z <= max.z) && "The minimum corner of the box must be less than or equal to maximum corner"' failed.
```
