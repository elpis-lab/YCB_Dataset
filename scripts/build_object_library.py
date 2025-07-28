"""Build URDF for objects.
Scripts from https://github.com/harvard-microrobotics/object2urdf
"""

import os
from object_builder import ObjectBuilder

# Build entire libraries of URDFs
# This is only suitable for objects built with single obj/stl file
# Models such as robots or articulated objects will not work properly
root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
object_folder = root_dir + "/ycb"

max_convex_hull = 4  # for coacd, default -1
decimate_face_count = 32  # for decimation, default -1 (no decimation)

builder = ObjectBuilder(
    object_folder,
    "_prototype.urdf",
    "_prototype.xml",
    "ycb_mass.json",
)
builder.build_library(
    force_overwrite=True,
    center="mass",  # object center is at mass center
    decompose_concave=True,  # build *_coacd.obj collision mesh
    force_decompose=True,  # overwrite decomposed files
    max_convex_hull=max_convex_hull,
    decimate_face_count=decimate_face_count,
)
