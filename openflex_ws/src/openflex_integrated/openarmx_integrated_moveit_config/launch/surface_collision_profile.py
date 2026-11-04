"""Build the MoveIt-only exterior collision profile from the authoritative xacro."""

from __future__ import annotations

import copy
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET


# None means every visual mesh on that link. These links have exterior meshes
# suitable for planning, but no authoritative simplified collision equivalent.
SURFACE_COLLISION_LINKS: dict[str, set[str] | None] = {
    "lift_base_link": None,
    "chest_link": None,
}

# Preserve the authoritative simplified geometry for the mobile shell and the
# articulated manipulation chain. In particular, the base box avoids adding
# wheel/steering detail, while arms and fingers retain their supplied meshes.
ORIGINAL_COLLISION_LINKS = {"base_link"}
for _side in ("left", "right"):
    for _index in range(8):
        ORIGINAL_COLLISION_LINKS.add(f"openarmx_{_side}_link{_index}")
    ORIGINAL_COLLISION_LINKS.add(f"openarmx_{_side}_hand")
    ORIGINAL_COLLISION_LINKS.add(f"openarmx_{_side}_left_finger")
    ORIGINAL_COLLISION_LINKS.add(f"openarmx_{_side}_right_finger")


def build_surface_collision_robot_description(
    authoritative_xacro: str, mappings: dict[str, str]
) -> str:
    """Return an expanded URDF whose collision geometry is exterior-only."""

    xacro = shutil.which("xacro")
    if xacro is None:
        raise RuntimeError("xacro executable is required to build the MoveIt collision profile")

    command = [xacro, authoritative_xacro]
    command.extend(f"{name}:={value}" for name, value in mappings.items())
    completed = subprocess.run(command, text=True, capture_output=True)
    if completed.returncode != 0:
        raise RuntimeError(
            "Failed to expand the authoritative xacro for the MoveIt collision profile:\n"
            + completed.stderr.strip()
        )
    root = ET.fromstring(completed.stdout)
    configured_links = set(SURFACE_COLLISION_LINKS) | ORIGINAL_COLLISION_LINKS
    found_links: set[str] = set()

    for link in root.findall("link"):
        link_name = link.attrib.get("name", "")
        original_collisions = list(link.findall("collision"))
        for collision in original_collisions:
            link.remove(collision)

        if link_name in ORIGINAL_COLLISION_LINKS:
            if not original_collisions:
                raise RuntimeError(f"No authoritative collision found for '{link_name}'")
            found_links.add(link_name)
            for collision in original_collisions:
                link.append(copy.deepcopy(collision))
            continue

        selected_meshes = SURFACE_COLLISION_LINKS.get(link_name)
        if link_name not in SURFACE_COLLISION_LINKS:
            continue
        found_links.add(link_name)
        inserted = 0
        for visual in link.findall("visual"):
            mesh = visual.find("./geometry/mesh")
            if mesh is None:
                continue
            filename = os.path.basename(mesh.attrib.get("filename", ""))
            if selected_meshes is not None and filename not in selected_meshes:
                continue
            collision = ET.Element("collision", {"name": f"moveit_surface_{inserted}"})
            origin = visual.find("origin")
            if origin is not None:
                collision.append(copy.deepcopy(origin))
            geometry = ET.SubElement(collision, "geometry")
            geometry.append(copy.deepcopy(mesh))
            link.append(collision)
            inserted += 1
        if inserted == 0:
            raise RuntimeError(f"No selected visual mesh found for collision link '{link_name}'")

    missing = configured_links - found_links
    if missing:
        raise RuntimeError(f"Collision profile links missing from expanded URDF: {sorted(missing)}")
    return ET.tostring(root, encoding="unicode")
