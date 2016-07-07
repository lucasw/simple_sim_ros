#!/usr/bin/env python
# Lucas Walter 2016

import rospy

from bullet_server.msg import Body, Constraint, Heightfield, Impulse
from urdf_parser_py.urdf import URDF


class LoadUrdf:
    def __init__(self):
        # self.param_name = rospy.get_param('~robot_description', 'robot_description')
        # TODO(lucasw) get xyz to spawn the root link at.
        self.robot = URDF.from_parameter_server()
        # rospy.loginfo(self.robot)

        # TODO(lucasw) make these services instead
        self.body_pub = rospy.Publisher("add_body", Body, queue_size=50)
        self.constraint_pub = rospy.Publisher("add_constraint", Constraint, queue_size=50)

        for link in self.robot.links:
            if rospy.is_shutdown():
                break
            print ' '
            rospy.loginfo(link.name)
            if link.collision is not None:
                body = Body()
                body.name = link.name
                body.pose.position.z = 2.0
                body.pose.orientation.w = 1.0
                # TODO(lwalter) is there a better way to get type?
                if str(type(link.collision.geometry)) == "<class 'urdf_parser_py.urdf.Box'>":
                    body.type = Body.BOX
                    body.scale.x = link.collision.geometry.size[0]
                    body.scale.y = link.collision.geometry.size[1]
                    body.scale.z = link.collision.geometry.size[2]
                if str(type(link.collision.geometry)) == "<class 'urdf_parser_py.urdf.Cylinder'>":
                    body.type = Body.CYLINDER
                    body.scale.x = link.collision.geometry.radius
                    body.scale.y = link.collision.geometry.radius
                    body.scale.z = link.collision.geometry.length

                self.body_pub.publish(body)
                rospy.sleep(2.0)

if __name__ == '__main__':
    rospy.init_node('load_urdf')
    load_urdf = LoadUrdf()
    # rospy.spin()
