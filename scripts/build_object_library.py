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

builder = ObjectBuilder(
    object_folder,
    "_prototype.urdf",
    "_prototype.xml",
    "ycb_mass.json",
    "vhacd",
)
builder.build_library(
    force_overwrite=True,
    center="mass",  # object center is at mass center
    decompose_concave=True,  # build *_vhacd.obj files
    force_decompose=False,  # overwrite decomposed files
    resolution=4000,  # resolution for vhacd (default 4e5)
    maxConvexHulls=16,  # max convex hulls for vhacd (default 64)
)
