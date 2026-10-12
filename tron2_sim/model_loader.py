"""MuJoCo model loading helpers."""

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np


def load_payload_model(model_path, mass, dimensions, offset):
    """Compile a rigid payload copy without modifying the asset submodule."""
    path = Path(model_path).resolve()
    dimensions = np.asarray(dimensions, dtype=float)
    offset = np.asarray(offset, dtype=float)
    if (dimensions.shape != (3,) or offset.shape != (3,)
            or not np.isfinite([mass, *dimensions, *offset]).all()
            or mass <= 0 or np.any(dimensions <= 0)):
        raise ValueError("Payload requires positive mass/dimensions and finite offset")
    root = ET.parse(path).getroot()
    if root.findall("include"):
        raise ValueError("Payload authoring requires a standalone MJCF")
    attachment = root.find(".//body[@name='gripper_base_Link']")
    if attachment is None or root.find(".//body[@name='paint_payload_Link']") is not None:
        raise ValueError("Payload attachment missing or payload already exists")
    compiler = root.find("compiler")
    mesh_dir = (path.parent / compiler.get("meshdir", "")).resolve()
    assets = {mesh.get("file"): (mesh_dir / mesh.get("file")).read_bytes()
              for mesh in root.findall("./asset/mesh")}
    compiler.set("meshdir", "")
    body = ET.SubElement(attachment, "body", name="paint_payload_Link",
                         pos=" ".join(map(str, offset)))
    x, y, z = dimensions
    inertia = mass / 12 * np.array([y*y+z*z, x*x+z*z, x*x+y*y])
    ET.SubElement(body, "inertial", pos="0 0 0", mass=str(mass),
                  diaginertia=" ".join(map(str, inertia)))
    ET.SubElement(body, "geom", name="paint_payload_geom", type="box",
                  size=" ".join(map(str, dimensions / 2)), density="0",
                  rgba="0.15 0.55 0.85 1")
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    for name in ("gripper_base_Link", "gripper1_Link", "gripper2_Link"):
        if root.find(f".//body[@name='{name}']") is not None:
            ET.SubElement(contact, "exclude", body1="paint_payload_Link", body2=name)
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"), assets)


def load_mujoco_model(model_path):
    """Load an MJCF file, including on Windows paths containing non-ASCII text.

    MuJoCo's native path loader can fail before parsing when a Windows parent
    directory contains non-ASCII characters.  Keep the normal path (which also
    supports includes) first, then fall back to the in-memory VFS interface.
    The upstream TRON2 assets use ``compiler meshdir`` with filename-only mesh
    references, so copying that directory into the VFS preserves semantics.
    """
    path = Path(model_path)
    try:
        return mujoco.MjModel.from_xml_path(str(path))
    except ValueError as error:
        if "Error opening file" not in str(error):
            raise

    root = ET.fromstring(path.read_text(encoding="utf-8"))
    compiler = root.find("compiler")
    mesh_dir_value = compiler.get("meshdir", "") if compiler is not None else ""
    mesh_dir = (path.parent / mesh_dir_value).resolve()
    assets = {
        mesh.relative_to(mesh_dir).as_posix(): mesh.read_bytes()
        for mesh in mesh_dir.rglob("*")
        if mesh.is_file()
    }
    if compiler is not None:
        compiler.set("meshdir", "")
    xml = ET.tostring(root, encoding="unicode")
    return mujoco.MjModel.from_xml_string(xml, assets)
