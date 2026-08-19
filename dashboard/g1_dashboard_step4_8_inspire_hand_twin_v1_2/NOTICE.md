# Third-party notices

## Unitree Robotics G1 model

The kinematic origins/axes used by the dashboard are derived from Unitree Robotics' G1 robot descriptions. `fetch_g1_assets.py` installs the official STL visual geometry from:

`unitreerobotics/unitree_mujoco/unitree_robots/g1/meshes`

The upstream Unitree repositories identify their model code/assets under the BSD-3-Clause license. The mesh files are not embedded in this ZIP; the included helper fetches/copies them into `static/model/g1/meshes` for local dashboard use.

## Three.js

`static/vendor/three.module.min.js`, `static/vendor/three.core.min.js`, and `static/vendor/STLLoader.js` are from Three.js r180.

Three.js license header: Copyright 2010-2025 Three.js Authors, SPDX-License-Identifier: MIT.

`STLLoader.js` has only been modified to resolve its `three` import to the locally vendored module instead of a package/bare-module import.


## Inspire RH56DFX visualization assets

Step 4.8 includes the user's local `xr_teleoperate_g1demo/assets/inspire_hand` URDF/STL files so the dashboard can render the exact installed hand model. These assets were supplied by the user from the robot filesystem; this package does not claim a new license for them. Preserve the upstream/project license notices when publishing or redistributing this snapshot.
