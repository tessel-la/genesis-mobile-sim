"""Convert pinned AWS visuals to textured GLB and derive conservative collision bounds.

The COLLADA exporter binds diffuse textures directly to image IDs. Restore those
bindings from XML, preserve UVs and normals, and embed the images in standard GLB.
Runtime boxes use bounds from the upstream collision proxies, preserving their
frame without mesh principal-axis alignment. Requires the simulator dependencies.
"""

import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

import collada
import numpy as np
import trimesh
from PIL import Image
from trimesh.visual.material import PBRMaterial

ROOT = Path(__file__).resolve().parents[1] / "assets/environments/warehouse"
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
bounds = {}

for path in sorted(ROOT.glob("models/*/meshes/*visual.DAE")):
    xml = ET.parse(path)
    images = {
        item.attrib["id"]: item.find("c:init_from", NS).text
        for item in xml.findall("c:library_images/c:image", NS)
    }
    effects = {item.attrib["id"]: item for item in xml.findall("c:library_effects/c:effect", NS)}
    materials = {
        item.attrib["id"]: effects[item.find("c:instance_effect", NS).attrib["url"][1:]]
        for item in xml.findall("c:library_materials/c:material", NS)
    }
    scene = trimesh.Scene()
    for geometry in collada.Collada(str(path)).scene.objects("geometry"):
        for primitive in geometry.primitives():
            if primitive.vertex_index is None or not primitive.vertex_index.size:
                continue
            vertices = primitive.vertex[primitive.vertex_index].reshape(-1, 3)
            normals = primitive.normal[primitive.normal_index].reshape(-1, 3)
            uv = primitive.texcoordset[0][primitive.texcoord_indexset[0]].reshape(-1, 2)
            diffuse = materials[primitive.material.id].find(".//c:diffuse", NS)
            texture, color = diffuse.find("c:texture", NS), diffuse.find("c:color", NS)
            material = PBRMaterial(metallicFactor=0.0, roughnessFactor=1.0)
            if texture is not None:
                image_path = (path.parent / images[texture.attrib["texture"]]).resolve()
                if not image_path.is_relative_to(path.parent.parent.resolve()):
                    raise ValueError(f"Texture is outside its warehouse model: {image_path}")
                with Image.open(image_path) as image:
                    material.baseColorTexture = image.convert("RGBA")
            elif color is not None:
                material.baseColorFactor = list(map(float, color.text.split()))
            mesh = trimesh.Trimesh(
                vertices=vertices,
                faces=np.arange(len(vertices)).reshape(-1, 3),
                vertex_normals=normals,
                process=False,
            )
            mesh.visual = trimesh.visual.TextureVisuals(uv=uv, material=material)
            scene.add_geometry(mesh)
    scene.apply_scale(0.01)
    scene.apply_transform(trimesh.transformations.rotation_matrix(-math.pi / 2, (1, 0, 0)))
    model = path.parent.parent.name
    output = ROOT / "visuals" / f"{model}.glb"
    output.parent.mkdir(exist_ok=True)
    output.write_bytes(scene.export(file_type="glb"))
    collision = trimesh.load(
        path.with_name(path.name.replace("_visual", "_collision")), force="scene"
    )
    limits = collision.bounds * 0.01
    bounds[model] = {
        "center": limits.mean(axis=0).tolist(),
        "size": (limits[1] - limits[0]).tolist(),
    }
    print(output.name)
(ROOT / "collision_bounds.json").write_text(json.dumps(bounds, indent=2) + "\n")
