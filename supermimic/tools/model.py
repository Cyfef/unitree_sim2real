"""Build the SuperMimic validation plant from the shared G1 model."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from contract import JOINT_NAMES

def load_model(path, hands, suspended=False, initial=None, backpack_mass=None):
    if not hands and backpack_mass is None:
        return mujoco.MjModel.from_xml_path(str(path))
    root = ET.parse(path).getroot()
    compiler = root.find("compiler")
    compiler.set("meshdir", str((path.parent / compiler.get("meshdir", "meshes")).resolve()))
    if backpack_mass is not None:
        torso=next(b for b in root.iter('body') if b.get('name')=='torso_link')
        backpack=ET.SubElement(torso,'body',name='backpack_link',pos='-0.094605 -0.0005 0.165324')
        inertia=np.array([.015510,.010909,.007288,.000017,.000563,.000059])*backpack_mass/1.9
        ET.SubElement(backpack,'inertial',pos='0 0 0',mass=str(backpack_mass),fullinertia=' '.join(map(str,inertia)))
        mesh=(path.parent/ 'meshes/backpack_link.STL').resolve()
        if not mesh.is_file():raise FileNotFoundError(mesh)
        ET.SubElement(root.find('asset'),'mesh',name='diagnostic_backpack_mesh',file=str(mesh))
        ET.SubElement(backpack,'geom',type='mesh',mesh='diagnostic_backpack_mesh',density='0')
    if not any(g.get("name") == "floor" for g in root.iter("geom")):
        ET.SubElement(root.find("worldbody"), "geom", name="floor", type="plane", size="0 0 .05")
    # Finger motors have their own controller on the real robot. This test
    # fixes their measured pose, while retaining their mass and collisions.
    ref = json.loads(Path(hands).read_text())
    if suspended:
        pelvis = next(b for b in root.iter("body") if b.get("name")=="pelvis")
        for joint in list(pelvis.findall("joint")):
            if joint.get("type")=="free": pelvis.remove(joint)
        # Removing the free joint welds pelvis to the world and changes
        # MuJoCo's parent-body collision filtering. Preserve the original
        # floating model's exclusions for the two direct hip-pitch children.
        contact = root.find("contact")
        if contact is None:
            contact = ET.SubElement(root, "contact")
        excluded = {frozenset((e.get("body1"), e.get("body2")))
                    for e in contact.findall("exclude")}
        for side in ("left", "right"):
            hip = side + "_hip_pitch_link"
            if frozenset(("pelvis", hip)) not in excluded:
                ET.SubElement(contact, "exclude", body1="pelvis", body2=hip)
        pelvis.set("pos","0 0 1.1")
        pelvis.set("quat"," ".join(str(q) for q in initial["pelvis_quaternion"]))
    fingers = ("thumb_0", "thumb_1", "thumb_2", "index_0", "index_1", "middle_0", "middle_1")
    equality = ET.SubElement(root, "equality")
    for side, values in ref["hands"].items():
        for name, q in zip(fingers, values):
            ET.SubElement(equality, "joint", joint1=f"{side}_hand_{name}_joint",
                          polycoef=f"{q} 0 0 0 0", solref=".002 1")
    # Match the joint loss parameters of the official MuJoCo baseline.
    for joint in root.iter("joint"):
        if joint.get("name") in set(JOINT_NAMES):
            joint.set("damping", ".05")
            joint.set("armature", ".01")
            joint.set("frictionloss", ".1" if "wrist" in joint.get("name") else ".2")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    return model

