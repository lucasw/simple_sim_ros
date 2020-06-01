#!/usr/bin/env python
# Lucas Walter 2016

import rospy

from bullet_server.msg import Body, Constraint, Heightfield, Impulse
from bullet_server.srv import *
from urdf_parser_py.urdf import URDF


class LoadUrdf:
    def __init__(self):
        # self.param_name = rospy.get_param('~robot_description', 'robot_description')
        # TODO(lucasw) get xyz to spawn the root link at.
        self.robot = URDF.from_parameter_server()
        # rospy.loginfo(self.robot)

        try:
            rospy.wait_for_service('add_compound', 5)
        except ROSException as e:
            rospy.logerr("no service available")
            rospy.logerr(e)
            return
        self.add_compound = rospy.ServiceProxy('add_compound', AddCompound)
        # self.body_pub = rospy.Publisher("add_body", Body, queue_size=50)
        # self.constraint_pub = rospy.Publisher("add_constraint", Constraint, queue_size=50)

        bodies = {}
        for link in self.robot.links:
            if rospy.is_shutdown():
                break
            # try:
            #     rospy.loginfo(link.visual.origin)
            # except:
            #     pass
            if link.collision is not None:
                body = Body()
                # TODO(lucasw) tf_prefix needed here?
                body.name = link.name
                body.pose.position.z = 2.0
                # TODO(lwalter) transformations.py rpy to quaternion
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

                bodies[body.name] = body
                # self.body_pub.publish(body)

        constraints = {}
        for joint in self.robot.joints:
            print ''
            print joint
            constraint = Constraint()
            constraint.body_a = joint.parent
            constraint.body_b = joint.child
            if joint.type == 'fixed':
                constraint.type = Constraint.FIXED
            constraints[joint.name] = constraint

        add_compound_request = AddCompoundRequest()
        for key in bodies.keys():
            # rospy.loginfo(bodies.name)
            add_compound_request.body.append(bodies[key])
        for key in constraints.keys():
            # rospy.loginfo(bodies.name)
            add_compound_request.constraint.append(constraints[key])

        try:
            add_compound_response = self.add_compound(add_compound_request)
            rospy.loginfo(add_compound_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('load_urdf')
    load_urdf = LoadUrdf()
    # rospy.spin()
