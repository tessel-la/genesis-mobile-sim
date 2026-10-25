# Warehouse assets

The shelf, pallet jack, and cardboard clutter models come from
[AWS RoboMaker small warehouse world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world),
commit `ee0af733315e78432408c3cd98d378ecee5f767c` (branch `ros2`).
Their original visual meshes, collision proxies, textures, and model descriptions
are retained unchanged under `models/`. The upstream permission notice is in
[LICENSE](LICENSE); per-file digests are in [SHA256SUMS.json](SHA256SUMS.json).

Source COLLADA units are centimeters. `scripts/prepare_warehouse_assets.py` preserves
UV coordinates and normals, repairs the exporter's direct image texture bindings,
and exports metric Y-up GLB visuals with embedded textures. The original files are
retained unchanged for provenance.

`collision_bounds.json` stores metric bounds derived from the original collision
proxies. Genesis uses these as conservative static boxes in the source model frame.
This avoids navigating under shelves or through pallet-jack handles and avoids
principal-axis alignment changing a static mesh's collision frame. Runtime does not
need COLLADA material repair or mesh collision decomposition.

The 18 × 12 m room, shelf placement, floor paint, and navigation test layout are
part of this integration. Six reused shelf models define a 2.7 m center aisle and
open routes around each end. Painted floor boundaries have no collisions.
