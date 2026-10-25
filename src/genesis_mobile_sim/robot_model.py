"""Use the bundled Go2 model unchanged in physics; add ROS sensor frames."""

from importlib.metadata import distribution
from pathlib import Path
from xml.etree import ElementTree as ET

BASE_FRAME = "base"
LASER_OFFSET = (0.0, 0.0, 0.20)
CAMERA_OFFSET = (0.34, 0.0, 0.04)


def model_path() -> Path:
    return Path(distribution("genesis-world").locate_file("genesis/assets/urdf/go2/urdf/go2.urdf"))


def robot_description() -> str:
    path = model_path()
    robot = ET.parse(path).getroot()
    for mesh in robot.findall(".//mesh"):
        relative = (path.parent / mesh.attrib["filename"]).resolve()
        mesh.set("filename", "package://go2/" + relative.relative_to(path.parents[1]).as_posix())
    for name, parent, position, rotation in (
        ("laser", BASE_FRAME, LASER_OFFSET, (0, 0, 0)),
        ("camera_link", BASE_FRAME, CAMERA_OFFSET, (0, 0, 0)),
        ("camera_optical_frame", "camera_link", (0, 0, 0), (-1.57079632679, 0, -1.57079632679)),
    ):
        ET.SubElement(robot, "link", name=name)
        joint = ET.SubElement(robot, "joint", name=f"{name}_sensor_joint", type="fixed")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=name)
        ET.SubElement(
            joint, "origin", xyz=" ".join(map(str, position)), rpy=" ".join(map(str, rotation))
        )
    return ET.tostring(robot, encoding="unicode")
