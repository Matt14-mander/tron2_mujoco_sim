"""MuJoCo model loading helpers."""

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


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
