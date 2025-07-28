"""Generate urdf for objects in a folder with a provided prototype urdf file.
Script from https://github.com/harvard-microrobotics/object2urdf

Modified to use CoACD for convex decomposition.
Added mass file to the urdf.
Also modified to generate mujoco xml files.
"""

from scipy.spatial.transform import Rotation
import numpy as np
import os
import copy
import trimesh

import coacd
import json
import xml.etree.ElementTree as ET


def run_coacd(filename, **kwargs):
    """
    Run convex decomposition with COACD

    For COACD, the result collision mesh
    is a concatenate of all the convex parts
    """
    mesh = trimesh.load(filename)
    mesh = coacd.Mesh(mesh.vertices, mesh.faces)

    # Run COACD decomposition
    decimate_face_count = kwargs.pop("decimate_face_count", -1)
    kwargs.pop("decimate_face_count", None)
    parts = coacd.run_coacd(mesh, **kwargs)

    # Run decimation and export each part
    out_files = []
    outfile = os.path.splitext(filename)[0]
    for i, (vertices, indices) in enumerate(parts):
        # convert back to trimesh format
        vertices = np.asarray(vertices, dtype=float)
        indices = np.asarray(indices, dtype=np.int64)
        convex = trimesh.Trimesh(vertices, indices, process=False)

        # decimate the convex parts
        if decimate_face_count > 0:
            convex = convex.simplify_quadric_decimation(
                face_count=decimate_face_count
            )

        outpath = f"{outfile}_coacd_{i}.stl"
        out_files.append(outpath)
        convex.export(outpath, file_type="stl")
    return out_files


class ObjectBuilder:
    def __init__(
        self,
        object_folder="",
        urdf_prototype="_prototype.urdf",
        xml_prototype="_prototype.xml",
        mass_file="ycb_mass.json",
    ):
        self.object_folder = os.path.abspath(object_folder)
        self.urdf_base = self._read_xml(urdf_prototype)
        self.xml_base = self._read_xml(xml_prototype)
        self.mass_data = self._load_mass_data(mass_file)

    # Recursively get all files with a specific extension, excluding a certain suffix
    def _get_files_recursively(
        self, start_directory, filter_extension=None, exclude_suffix=None
    ):
        for root, dirs, files in os.walk(start_directory):
            for file in files:
                if filter_extension is None or file.lower().endswith(
                    filter_extension
                ):
                    if isinstance(exclude_suffix, str):
                        if not exclude_suffix in file.lower():
                            yield (
                                root,
                                file,
                                os.path.abspath(os.path.join(root, file)),
                            )

    # Read and parse a URDF from a file
    def _read_xml(self, filename):
        root = ET.parse(filename).getroot()
        return root

    # Convert a list to a space-separated string
    def _list2str(self, in_list):
        out = ""
        for el in in_list:
            out += str(el) + " "
        return out[:-1]

    # Convert a space-separated string to a list
    def _str2list(self, in_str):
        out = in_str.split(" ")
        out = [float(el) for el in out]
        return out

    # Load mass data from file
    def _load_mass_data(self, mass_file):
        if mass_file is None:
            return {}

        if not os.path.exists(mass_file):
            print(
                f"Warning: Mass file {mass_file} not found, using default masses"
            )
            return {}

        try:
            with open(mass_file, "r") as f:
                return json.load(f)
        except json.JSONDecodeError:
            print(
                f"Warning: Invalid JSON in mass file {mass_file}, using default masses"
            )
            return {}

    # Find the center of mass of the object
    def get_center_of_mass(self, filename):
        mesh = trimesh.load(filename)
        if isinstance(mesh, trimesh.Scene):
            print(
                "Imported combined mesh: using centroid rather than center of mass"
            )
            return mesh.centroid
        else:
            return mesh.center_mass

    # Find the geometric center of the object
    def get_geometric_center(self, filename):
        mesh = trimesh.load(filename)
        return copy.deepcopy(mesh.centroid)

    # Get the middle of a face of the bounding box
    def get_face(self, filename, edge):
        mesh = trimesh.load(filename)
        bounds = mesh.bounds
        face = copy.deepcopy(mesh.centroid)
        if edge in ["top", "xy_pos"]:
            face[2] = bounds[1][2]
        elif edge in ["bottom", "xy_neg"]:
            face[2] = bounds[0][2]
        elif edge in ["xz_pos"]:
            face[1] = bounds[1][1]
        elif edge in ["xz_neg"]:
            face[1] = bounds[0][1]
        elif edge in ["yz_pos"]:
            face[0] = bounds[1][0]
        elif edge in ["yz_neg"]:
            face[0] = bounds[0][0]

        return face

    # Find the center of mass of the object
    def save_to_obj(self, filename):
        name, ext = os.path.splitext(filename)
        obj_filename = name + ".obj"
        mesh = trimesh.load(filename)
        mesh.export(obj_filename)
        return obj_filename

    # Replace an attribute in a feild of a URDF
    def replace_urdf_attribute(self, urdf, feild, attribute, value):
        urdf = self.replace_urdf_attributes(urdf, feild, {attribute: value})
        return urdf

    # Replace several attributes in a feild of a URDF
    def replace_urdf_attributes(
        self, urdf, feild, attribute_dict, sub_feild=None
    ):

        if sub_feild is None:
            sub_feild = []

        field_obj = urdf.find(feild)

        if field_obj is not None:
            if len(sub_feild) > 0:
                for child in reversed(sub_feild):
                    field_obj = ET.SubElement(field_obj, child)
            field_obj.attrib.update(attribute_dict)
            # field_obj.attrib = attribute_dict
        else:
            feilds = feild.split("/")
            new_feild = "/".join(feilds[0:-1])
            sub_feild.append(feilds[-1])
            self.replace_urdf_attributes(
                urdf, new_feild, attribute_dict, sub_feild
            )

    # Make an updated copy of the URDF for the current object
    def update_urdf(
        self,
        object_file,
        object_name,
        collision_files,
        mass_center=None,
        mass_value=None,
    ):
        # Update the filenames and object name
        new_urdf = copy.deepcopy(self.urdf_base)
        new_urdf.attrib["name"] = object_name
        self.replace_urdf_attribute(
            new_urdf, ".//visual/geometry/mesh", "filename", object_file
        )

        # Update mass if provided
        if mass_value is not None:
            self.replace_urdf_attribute(
                new_urdf, ".//inertial/mass", "value", str(mass_value)
            )

        # Output the center of mass if provided
        if mass_center is not None:
            # Check if there's a geometry offset
            offset_ob = new_urdf.find(".//collision/origin")
            if offset_ob is not None:
                offset_str = offset_ob.attrib.get("xyz", "0 0 0")
                rot_str = offset_ob.attrib.get("rpy", "0 0 0")
                offset = self._str2list(offset_str)
                rpy = self._str2list(rot_str)
            else:
                offset = [0, 0, 0]
                rpy = [0, 0, 0]

            # Check if there's a scale factor and apply it
            scale_ob = new_urdf.find(".//collision/geometry/mesh")
            if scale_ob is not None:
                scale_str = scale_ob.attrib.get("scale", "1 1 1")
                scale = self._str2list(scale_str)
            else:
                scale = [1, 1, 1]

            for idx, axis in enumerate(mass_center):
                mass_center[idx] = -mass_center[idx] * scale[idx] + offset[idx]

            rot = Rotation.from_euler("xyz", rpy)
            rot_matrix = rot.as_matrix()
            mass_center = np.matmul(
                rot_matrix, np.vstack(np.asarray(mass_center))
            ).squeeze()

            self.replace_urdf_attributes(
                new_urdf,
                ".//visual/origin",
                {
                    "xyz": self._list2str(mass_center),
                    "rpy": self._list2str(rpy),
                },
            )
            self.replace_urdf_attributes(
                new_urdf,
                ".//collision/origin",
                {
                    "xyz": self._list2str(mass_center),
                    "rpy": self._list2str(rpy),
                },
            )

        # Update the collision mesh files
        self.replace_urdf_attribute(
            new_urdf,
            ".//collision/geometry/mesh",
            "filename",
            collision_files[0],
        )
        # Copy collision sections and replace mesh files one by one
        col = new_urdf.find(".//collision")
        parent = new_urdf.find(".//link")
        for collision_file in collision_files[1:]:
            clone = copy.deepcopy(col)
            clone.find("geometry/mesh").attrib["filename"] = collision_file
            parent.insert(list(parent).index(col) + 1, clone)
            col = clone
        return new_urdf

    # Make an updated copy of the Mujoco XML for the current object
    def update_xml(
        self,
        object_file,
        object_name,
        collision_files,
        mass_center=None,
        mass_value=None,
    ):
        # Update the filenames and object name
        new_xml = copy.deepcopy(self.xml_base)
        new_xml.attrib["model"] = object_name

        # Update mesh file names in assets
        visual_mesh = new_xml.find(".//asset/mesh[@name='_name_visual_mesh']")
        if visual_mesh is not None:
            visual_mesh.attrib["name"] = f"{object_name}_visual_mesh"
            visual_mesh.attrib["file"] = object_file

        collision_mesh = new_xml.find(
            ".//asset/mesh[@name='_name_collision_mesh']"
        )
        if collision_mesh is not None:
            collision_mesh.attrib["name"] = f"{object_name}_collision_mesh_0"
            collision_mesh.attrib["file"] = collision_files[0]
            # Copy collision sections and replace mesh files one by one
            parent = new_xml.find("asset")
            for i, collision_file in enumerate(collision_files[1:]):
                clone = copy.deepcopy(collision_mesh)
                clone.attrib["name"] = f"{object_name}_collision_mesh_{i + 1}"
                clone.attrib["file"] = collision_file
                parent.insert(
                    list(parent).index(collision_mesh) + i + 1, clone
                )

        # Update texture and material names
        texture = new_xml.find(".//asset/texture[@name='_name_texture']")
        if texture is not None:
            texture.attrib["name"] = f"{object_name}_texture"
            # Assume texture file is the same name in _prototype.xml
            texture_file = object_file.replace(
                os.path.basename(object_file), texture.attrib["file"]
            )
            texture.attrib["file"] = texture_file

        material = new_xml.find(".//asset/material[@name='_name_material']")
        if material is not None:
            material.attrib["name"] = f"{object_name}_material"
            material.attrib["texture"] = f"{object_name}_texture"

        # Update body name
        body = new_xml.find(".//worldbody/body[@name='_name']")
        if body is not None:
            body.attrib["name"] = object_name

        # Update joint name
        joint = new_xml.find(".//freejoint[@name='_name_joint']")
        if joint is not None:
            joint.attrib["name"] = f"{object_name}_joint"

        # Update geom names and mesh references
        visual_geom = new_xml.find(".//geom[@name='_name_visual_geom']")
        if visual_geom is not None:
            visual_geom.attrib["name"] = f"{object_name}_visual_geom"
            visual_geom.attrib["mesh"] = f"{object_name}_visual_mesh"
            visual_geom.attrib["material"] = f"{object_name}_material"

        collision_geom = new_xml.find(".//geom[@name='_name_collision_geom']")
        if collision_geom is not None:
            collision_geom.attrib["name"] = f"{object_name}_collision_geom_0"
            collision_geom.attrib["mesh"] = f"{object_name}_collision_mesh_0"

        # Update mass if provided
        inertial = new_xml.find(".//inertial")
        if inertial is not None and mass_value is not None:
            inertial.attrib["mass"] = str(mass_value)

        # Update position if mass center is provided
        if mass_center is not None:
            mass_center_str = (
                f"{mass_center[0]} {mass_center[1]} {mass_center[2]}"
            )

            # Update visual geom position
            if visual_geom is not None:
                visual_geom.attrib["pos"] = mass_center_str
            # Update collision geom position
            if collision_geom is not None:
                collision_geom.attrib["pos"] = mass_center_str

        # Update the collision mesh files
        # Copy collision sections and replace mesh files one by one
        parent = new_xml.find(".//worldbody/body")
        for i, collision_file in enumerate(collision_files[1:]):
            clone = copy.deepcopy(collision_geom)
            clone.attrib["name"] = f"{object_name}_collision_geom_{i + 1}"
            clone.attrib["mesh"] = f"{object_name}_collision_mesh_{i + 1}"
            parent.insert(list(parent).index(collision_geom) + i + 1, clone)

        return new_xml

    # Save a file to a file
    def save_file(self, new_file, filename, overwrite=False):
        out_file = os.path.join(self.object_folder, filename)

        # Do not overwrite the file unless the option is True
        if os.path.exists(out_file) and not overwrite:
            return

        # Save the file
        mydata = ET.tostring(new_file)
        with open(out_file, "wb") as f:
            f.write(mydata)

    # Build a URDF from an object file
    def build_description(
        self,
        filename,
        output_folder=None,
        force_overwrite=False,
        decompose_concave=False,
        force_decompose=False,
        center="mass",
        **kwargs,
    ):

        # If no output folder is specified, use the base object folder
        if output_folder is None:
            output_folder = self.object_folder

        # Generate a relative path from the output folder to the geometry files
        filename = os.path.abspath(filename)
        common = os.path.commonprefix([output_folder, filename])
        rel = os.path.join(filename.replace(common, ""))
        if rel[0] == os.path.sep:
            rel = rel[1:]

        name = rel.split(os.path.sep)[0]
        rel = rel.replace(os.path.sep, "/")
        file_name_raw, file_extension = os.path.splitext(filename)

        # Get mass data
        mass_value = self.mass_data.get(name, None)

        # Calculate the center of mass
        if center == "mass":
            mass_center = self.get_center_of_mass(filename)

        elif center == "geometric":
            mass_center = self.get_geometric_center(filename)

        elif center in [
            "top",
            "bottom",
            "xy_pos",
            "xy_neg",
            "xz_pos",
            "xz_neg",
            "yz_pos",
            "yz_neg",
        ]:
            mass_center = self.get_face(filename, center)

        else:
            mass_center = None

        # If the user wants to run convex decomposition on concave objects
        if decompose_concave:
            if file_extension == ".stl":
                obj_filename = self.save_to_obj(filename)
                visual_file = rel.replace(file_extension, ".obj")
            elif file_extension == ".obj":
                obj_filename = filename
                visual_file = rel
            else:
                raise ValueError(
                    "Your filetype needs to be an STL or OBJ"
                    + "to perform concave decomposition"
                )

            # Only run a decomposition if one does not exist,
            # or if the user forces an overwrite
            check_file = obj_filename.replace(".obj", "_" + "coacd_0" + ".stl")
            if not os.path.exists(check_file) or force_decompose:
                collision_paths = run_coacd(obj_filename, **kwargs)
                collision_files = [
                    os.path.relpath(path, output_folder)
                    for path in collision_paths
                ]
        else:
            collision_files = [visual_file]

        urdf_out = self.update_urdf(
            visual_file,
            name,
            collision_files=collision_files,
            mass_center=mass_center,
            mass_value=mass_value,
        )
        self.save_file(urdf_out, name + ".urdf", force_overwrite)
        xml_out = self.update_xml(
            visual_file,
            name,
            collision_files=collision_files,
            mass_center=mass_center,
            mass_value=mass_value,
        )
        self.save_file(xml_out, name + ".xml", force_overwrite)

    # Build the URDFs for all objects in your library.
    def build_library(self, **kwargs):
        print("\nFOLDER: %s" % (self.object_folder))

        # Get all OBJ files
        obj_files = self._get_files_recursively(
            self.object_folder,
            filter_extension=".obj",
            exclude_suffix="coacd",
        )
        stl_files = self._get_files_recursively(
            self.object_folder,
            filter_extension=".stl",
            exclude_suffix="coacd",
        )

        obj_folders = []
        for root, _, full_file in obj_files:
            common = os.path.commonprefix([self.object_folder, full_file])
            rel = os.path.join(full_file.replace(common, ""))
            print("\tBuilding: %s" % (rel))

            obj_folders.append(root)
            self.build_description(full_file, **kwargs)

        for root, _, full_file in stl_files:
            if root not in obj_folders:
                common = os.path.commonprefix([self.object_folder, full_file])
                rel = os.path.join(full_file.replace(common, ""))
                print("Building: %s" % (rel))

                self.build_description(full_file, **kwargs)
