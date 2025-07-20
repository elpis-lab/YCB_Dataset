"""Used to decompose objects into near convex parts with V-HACD"""

import trimesh
import os
import coacd
import numpy as np


def do_vhacd(filename, outfile, debug=False, **kwargs):
    try:
        mesh = trimesh.load(filename)
        convex_list = mesh.convex_decomposition(**kwargs)
        convex = trimesh.util.concatenate(convex_list)
        convex.export(outfile)

    except ValueError:
        print("No direct VHACD backend available, trying pybullet")

        try:
            import pybullet as p

            p.vhacd(filename, outfile, "vhacd_log.txt", **kwargs)
        except ModuleNotFoundError:
            print(
                +"\nERROR - pybullet module not found: "
                + "If you want to do convex decomposisiton, "
                + "make sure you install pybullet (https://pypi.org/project/pybullet) "
                + "or install VHACD directly (https://github.com/mikedh/trimesh/issues/404)"
                + "\n"
            )
            raise


def do_coacd(filename, outfile, debug=False, **kwargs):
    mesh = trimesh.load(filename)
    mesh = coacd.Mesh(mesh.vertices, mesh.faces)
    parts = coacd.run_coacd(mesh, **kwargs)
    convex_list = []
    for _, (vertices, indices) in enumerate(parts):
        vertices = np.asarray(vertices, dtype=float)
        indices = np.asarray(indices, dtype=np.int64)
        convex = trimesh.Trimesh(vertices, indices, process=False)
        convex_list.append(convex)
    convex = trimesh.util.concatenate(convex_list)
    convex.export(outfile)


def save_to_obj(file_name):
    """Save stl file into obj files"""
    name, ext = os.path.splitext(file_name)
    obj_file_name = name + ".obj"
    mesh = trimesh.load(file_name)
    mesh.export(obj_file_name)
    return obj_file_name


def decompose(file_name, force_decompose, method="coacd", **kwargs):
    """Decompose objects into near convex parts"""
    file_extension = os.path.splitext(file_name)[1]
    if file_extension == ".stl":
        obj_file_name = save_to_obj(file_name)
    elif file_extension == ".obj":
        obj_file_name = file_name
    else:
        raise ValueError(
            "Needs to be an STL or OBJ to perform concave decomposition"
        )

    # Only run a decomposition if one does not exist, or if the user forces an overwrite
    out_file = obj_file_name.replace(".obj", "_" + method + ".stl")
    if not os.path.exists(out_file) or force_decompose:
        if method == "vhacd":
            do_vhacd(obj_file_name, out_file, **kwargs)
        elif method == "coacd":
            do_coacd(obj_file_name, out_file, **kwargs)
        else:
            raise ValueError("Invalid method: " + method)
    else:
        print("Skip as it already exists and overwriting is disabled")


def main():
    # Perform concave decomposition of the entire library
    # Loop for subfolders
    force_decompose = True
    method = "vhacd"
    vhacd_resolution = 4000  # for vhacd, default 4e5
    vhacd_max_convex_hull = 16  # for vhacd, default 64
    coacd_resolution = 2000  # for coacd, default 2000
    coacd_preprocess_resolution = 30  # for coacd, default 30
    coacd_max_convex_hull = 8  # for coacd, default -1

    # Convex decomposition in the root directory
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    object_folder = "ycb"

    curr_dir = root_dir + "/" + object_folder + "/"
    for subdir in os.listdir(curr_dir):
        curr_path = os.path.join(curr_dir, subdir)
        if not os.path.isdir(curr_path):
            continue

        # Loop through all the model obj/stl files
        for f in os.listdir(curr_path):
            curr_file = os.path.join(curr_path, f)

            # Skip non stl/obj files
            if not os.path.isfile(curr_file) or not (
                f.endswith(".stl") or f.endswith(".obj")
            ):
                continue
            # Skip the files that ends with suffix
            if f.endswith(method + ".stl") or f.endswith(method + ".obj"):
                continue

            print("Decomposing", subdir)
            file_name = os.path.join(curr_path, f)
            decompose(
                file_name,
                force_decompose,
                method=method,
                resolution=vhacd_resolution,
                maxConvexHulls=vhacd_max_convex_hull,
                # resolution=coacd_resolution,
                # preprocess_resolution=coacd_preprocess_resolution,
                # max_convex_hull=coacd_max_convex_hull,
            )


if __name__ == "__main__":
    main()
