"""Verify the installed original robot, its ROS description, and mesh boundary."""

import hashlib
import json
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import urlopen
from xml.etree import ElementTree as ET

from genesis_mobile_sim.mesh_server import serve_meshes
from genesis_mobile_sim.policy import JOINT_NAMES
from genesis_mobile_sim.robot_model import model_path, robot_description

original = ET.parse(model_path()).getroot()
robot = ET.fromstring(robot_description())
assert len(robot.findall("link")) == len(original.findall("link")) + 3
assert all(robot.find(f"joint[@name='{name}']") is not None for name in JOINT_NAMES)
meshes = robot.findall(".//mesh")
assert len(meshes) == len(original.findall(".//mesh")) > 0
with serve_meshes(host="127.0.0.1", port=0) as server:
    root = f"http://127.0.0.1:{server.server_port}/"
    for resource in {mesh.attrib["filename"] for mesh in meshes}:
        with urlopen(root + resource.removeprefix("package://"), timeout=5) as response:
            assert response.status == 200 and response.read()
    for resource in ("", "go2/urdf/go2.urdf", "../etc/passwd", "go2/../../etc/passwd"):
        try:
            urlopen(root + resource, timeout=5)
        except HTTPError as error:
            assert error.code == 404
        else:
            raise AssertionError(f"Unexpected access to {resource!r}")
print(
    {
        "existing_robot_links": len(original.findall("link")),
        "meshes": len(meshes),
        "resource_boundary": "passed",
    }
)

asset_root = Path(__file__).resolve().parents[1] / "assets/environments/warehouse"
for relative, digest in json.loads((asset_root / "SHA256SUMS.json").read_text()).items():
    assert hashlib.sha256((asset_root / relative).read_bytes()).hexdigest() == digest, relative
print({"warehouse_asset_integrity": "passed"})
