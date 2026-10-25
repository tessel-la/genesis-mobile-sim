"""Warehouse navigation course using pinned AWS RoboMaker model assets."""

import json
import math
from pathlib import Path

ASSETS = Path(__file__).resolve().parents[2] / "assets/environments/warehouse/models"
# A 2.7 m center aisle and open end routes connect receiving and inspection areas.
SHELVES = tuple((x, y, 0.0) for y in (-1.8, 1.8) for x in (-4.5, 0.0, 4.5))


def warehouse(scene, gs):
    scene.add_entity(gs.morphs.Plane(), surface=gs.surfaces.Default(color=(0.48, 0.50, 0.52, 1)))
    for position, size in (
        ((9, 0, 1.5), (0.18, 12, 3)),
        ((-9, 0, 1.5), (0.18, 12, 3)),
        ((0, 6, 1.5), (18, 0.18, 3)),
        ((0, -6, 1.5), (18, 0.18, 3)),
    ):
        scene.add_entity(
            gs.morphs.Box(pos=position, size=size, fixed=True),
            surface=gs.surfaces.Default(color=(0.79, 0.81, 0.80, 1)),
        )
    # Painted boundaries have no collision geometry and do not pollute lidar maps.
    for y in (-1.15, 1.15, -4.5, 4.5):
        scene.add_entity(
            gs.morphs.Box(pos=(0, y, 0.002), size=(16, 0.06, 0.003), fixed=True, collision=False),
            surface=gs.surfaces.Default(color=(0.95, 0.72, 0.10, 1)),
        )
    for x in (-7.5, 7.5):
        for y in (-3.5, 3.5):
            scene.add_entity(
                gs.morphs.Box(
                    pos=(x, y, 0.003), size=(1.4, 0.06, 0.004), fixed=True, collision=False
                ),
                surface=gs.surfaces.Default(color=(0.95, 0.72, 0.10, 1)),
            )

    bounds = json.loads((ASSETS.parent / "collision_bounds.json").read_text())

    def prop(name, position, yaw=0):
        model = "aws_robomaker_warehouse_" + name
        scene.add_entity(
            gs.morphs.Mesh(
                file=str(ASSETS.parent / "visuals" / f"{model}.glb"),
                pos=position,
                euler=(0, 0, yaw),
                fixed=True,
                collision=False,
            )
        )
        # Static navigation obstacles use conservative bounds from the original
        # collision mesh. Native boxes preserve the asset frame exactly.
        x, y, z = bounds[model]["center"]
        angle = math.radians(yaw)
        center = (
            position[0] + math.cos(angle) * x - math.sin(angle) * y,
            position[1] + math.sin(angle) * x + math.cos(angle) * y,
            position[2] + z,
        )
        scene.add_entity(
            gs.morphs.Box(
                pos=center,
                size=bounds[model]["size"],
                euler=(0, 0, yaw),
                fixed=True,
                visualization=False,
            )
        )

    for position in SHELVES:
        prop("ShelfD_01", position)
    prop("PalletJackB_01", (7, -0.3, 0), 90)
    prop("PalletJackB_01", (-6.8, -3.6, 0), -90)
    prop("ClutteringA_01", (5.5, 4.2, 0))
    prop("ClutteringA_01", (-3.0, -4.0, 0), 90)
