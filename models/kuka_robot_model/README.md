# Robot model files

This directory contains the URDF and STL geometry used by the MATLAB visualization scripts.

- `urdf/kuka_robot_model.urdf`: original exported URDF.
- `urdf/robot_fixed.urdf`: URDF using link-frame-corrected meshes.
- `meshes/`: original exported meshes.
- `meshes_linkframe/`: meshes rebased for the corrected URDF.

The MATLAB script `matlab/fix_meshes.m` regenerates the corrected mesh set and rewrites the URDF mesh paths.

Before publishing these assets, confirm that you have the right to redistribute the CAD-derived geometry. KUKA and KR 16-2 are trademarks or product names of their respective owner; this student project is not affiliated with or endorsed by KUKA.
