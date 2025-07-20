# YCB_Dataset

YCB dataset with urdf and xml files. Ready to be used in different simulations such as [PyBullet](https://github.com/bulletphysics/bullet3), [Genesis](https://github.com/Genesis-Embodied-AI/Genesis) and [Mujoco](https://github.com/google-deepmind/mujoco).

## Procedure

Here is the procedure of how this repository is made.

- The dataset is downloaded from the [YCB dataset website](https://www.ycbbenchmarks.com/) with script `download_ycb_dataset.py` provided by [ycb-tools](https://github.com/sea-bass/ycb-tools/tree/main). The data is cleaned to only keep the `textured.mtl`, `textured.obj` and `texture_map.png` of Google 16K models with `clean_ycb_dataset.py`.
- The urdf files are generated with scripts `build_object_library.py` and `object_builder.py` based on [object2urdf](https://github.com/harvard-microrobotics/object2urdf). Convex decomposition is done with VHACD using `trimesh`.
- You can always rerun the convex decomposition to get the satisfy results with `convex_decomposition.py`. The current setting may not be suitable for all objects. There is also an option to use [CoACD](https://github.com/SarahWeiii/CoACD) instead.
- The mass of each object is set based on the provided mass [form](http://www.ycbbenchmarks.com/wp-content/uploads/2015/09/object-list-Sheet1.pdf) (also stored in `ycb_mass.json`). The inertia of all objects are simply diagonal matrix with value 1e-3.
- All the texture images are compressed to size of 1024x1024 using `image_compressor.py`.

## Models not included

This repository only includes models with Google 16K. The other models, shown as below, are not included.

- models that don'y have google_16k:
  - 001, 023, 039, 041, 046, 047, 049, 063-c/d/e/f, 072-f/g/h/i/j/k, 073-h/i/j/k/l/m, 076
