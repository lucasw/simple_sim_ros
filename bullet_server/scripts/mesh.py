#!/usr/bin/env python

import rospy

rospy.init_node("stl")

file_name = rospy.get_param("~file_name")

if False:
    import vtk
    reader = vtk.vtkSTLReader()
    reader.SetFileName(fil_name)
    reader.Update()
    polydata = reader.GetOutput()
    print polydata.GetPoints()
    print polydata.GetPolys()

from pyassimp import pyassimp

scene = pyassimp.load(file_name)
print scene
mesh = scene.meshes[0]

print dir(mesh)
print mesh.faces[0].indices
# print mesh.vertices
# print mesh.

# TODO(lucasw) convert the vertices and indices to a shape_msg/Mesh
