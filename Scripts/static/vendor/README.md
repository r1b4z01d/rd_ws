# Vendored browser libraries for the 3D view (`/3d`, `Scripts/map3d.html`)

Copied unmodified so the robot can serve the page with no internet access:

| path | package | version | license |
|---|---|---|---|
| `three.module.min.js`, `three.core.min.js`, `jsm/controls/OrbitControls.js`, `jsm/loaders/{STLLoader,ColladaLoader,TGALoader}.js` | [three](https://github.com/mrdoob/three.js) | 0.182.0 | MIT (`LICENSE.three`) |
| `urdf/URDFLoader.js`, `urdf/URDFClasses.js` | [urdf-loader](https://github.com/gkjohnson/urdf-loaders) | 0.12.6 | Apache-2.0 |

The page maps the bare specifiers with an import map (`three` -> `three.module.min.js`, `three/examples/jsm/` -> `jsm/`).
To upgrade, copy the same files from a `node_modules` tree of the new versions.
