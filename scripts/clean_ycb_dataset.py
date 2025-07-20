import os
import shutil
import re

# Define an output folder
root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
object_folder = "ycb"

# Keep only the google_16k folder, textured.mtl, textured.obj and texture_map.png
files_to_keep = ["textured.mtl", "textured.obj", "texture_map.png"]

# Loop through all the object folders
curr_dir = root_dir + "/" + object_folder + "/"
obj_dirs = os.listdir(curr_dir)
for obj_dir in obj_dirs:
    obj_path = os.path.join(curr_dir, obj_dir)
    if not os.path.isdir(obj_path):
        continue

    # Find the google_16k folder
    mesh_dirs = []
    google_mesh_path = None
    for meshdir in os.listdir(obj_path):
        mesh_path = os.path.join(obj_path, meshdir)
        mesh_dirs.append(mesh_path)
        if meshdir == "google_16k":
            google_mesh_path = mesh_path

    # No google_16k folder, delete this object folder
    if google_mesh_path is None:
        shutil.rmtree(obj_path, ignore_errors=True)
        continue

    # Move textured.mtl, textured.obj and texture_map.png out to the object folder
    for f in files_to_keep:
        src = os.path.join(google_mesh_path, f)
        dst = os.path.join(obj_path, f)
        shutil.move(src, dst)

    # Then delete all the meshdir
    for meshdir in mesh_dirs:
        shutil.rmtree(meshdir, ignore_errors=True)

    # Rename the object folder to remove the leading numerical prefix
    m = re.match(r"^(\d+)[-_ ]*(.*)$", obj_dir)
    new_name = m.group(2)
    os.rename(obj_path, os.path.join(curr_dir, new_name))
