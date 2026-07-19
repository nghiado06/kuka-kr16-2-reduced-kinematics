# Before publishing this repository

Resolve the following technical and legal points before making the repository public:

1. **Active degree-of-freedom count**  
   The current URDF contains three revolute joints (`joint_0`, `joint_1`, and `joint_2`) plus a fixed end-tip joint. The MATLAB scripts also solve for `theta_1`, `theta_2`, and `theta_3`. Describe the repository as a reduced 3R model unless a fourth active joint is added.

2. **Tool-offset consistency**  
   The code currently contains both `476.5 mm` and `500 mm` tool-offset values. Select the value that matches the final CAD/URDF model and centralize it in one parameter file before publishing numerical results.

3. **MATLAB version**  
   Add the exact MATLAB release used for validation.

4. **Asset rights**  
   Confirm redistribution rights for the STL and URDF assets. If uncertain, publish the MATLAB source code without the geometry and provide instructions for users to supply their own model files.

5. **Demo media**  
   Add a GIF or image to `media/` and replace the placeholders in the main README.

6. **Reproducibility check**  
   Clone the repository into a clean directory and run the documented workflow from start to finish.
