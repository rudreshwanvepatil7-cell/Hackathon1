import os
import sys
import re
import mujoco

cwd = os.getcwd()
sys.path.append(cwd)

from rl.utils import crab_constants as consts  # noqa: E402


def fix_actuators(xml_str: str) -> str:
    # Note: this is extremely hacky lol
    def fix_actuator(act_tag: re.Match[str]) -> str:
        act_tag = act_tag.group()
        name = re.search(r"(?<=name=\")[^\"]*(?=\")", act_tag).group()
        act_class = name[name.rfind("_") + 1 :]
        return f'<position name="{name}" joint="{name}" kp="{consts.kp[act_class]}" dampratio="1" />'

    return re.sub(r"<general[^/>]*/>", fix_actuator, xml_str)


def add_sensors(xml_str: str) -> str:
    # This is also hacky but mujoco is broken
    return xml_str.replace(
        "<touch/>",
        """<gyro site="imu" name="gyro"/>
    <velocimeter site="imu" name="local_linvel"/>
    <accelerometer site="imu" name="accelerometer"/>
    <framepos objtype="site" objname="imu" name="position"/>
    <framezaxis objtype="site" objname="imu" name="upvector"/>
    <framexaxis objtype="site" objname="imu" name="forwardvector"/>
    <framelinvel objtype="site" objname="imu" name="global_linvel"/>
    <frameangvel objtype="site" objname="imu" name="global_angvel"/>
    <framequat objtype="site" objname="imu" name="orientation"/>
    <framelinvel objtype="site" objname="front_right" name="front_right_global_linvel"/>
    <framelinvel objtype="site" objname="front_left" name="front_left_global_linvel"/>
    <framelinvel objtype="site" objname="back_right" name="back_right_global_linvel"/>
    <framelinvel objtype="site" objname="back_left" name="back_left_global_linvel"/>
    <framepos objtype="site" objname="front_right" name="front_right_pos" reftype="site" refname="imu"/>
    <framepos objtype="site" objname="front_left" name="front_left_pos" reftype="site" refname="imu"/>
    <framepos objtype="site" objname="back_right" name="back_right_pos" reftype="site" refname="imu"/>
    <framepos objtype="site" objname="back_left" name="back_left_pos" reftype="site" refname="imu"/>""",
    )


def change_geom_types(xml_str: str) -> str:
    # currently cylinder collisions not implemented in mjx
    return xml_str.replace('"cylinder"', '"capsule"')


def fix_assets(xml_str: str) -> str:
    return xml_str.replace("base_link.stl", "base_link_simple.stl")


def make_crab_xml() -> None:
    xml_path = consts.ROOT_PATH / "crab.urdf"

    # urdf_str = fix_urdf_paths(xml_path)

    urdf_str = xml_path.read_text().replace("package://", str(consts.ROOT_PATH) + "/")

    # model = mujoco.MjModel.from_xml_string(urdf_str)
    spec = mujoco.MjSpec.from_string(urdf_str)
    spec.compile()

    base_link = [body for body in spec.bodies if body.name == "base_link"][0]
    base_link.add_site(name="imu", pos=[0, 0, 0])

    # spec.compile()

    wrist_links = [body for body in spec.bodies if body.name.endswith("wrist_link")]
    for wl in wrist_links:
        site_name = re.sub(r"__.*", "", wl.name)  # cut off at "__", e.g. "back_right"
        wl.add_site(name=site_name, pos=[0, 0, 0.15])

    # temp sensor
    spec.add_sensor()

    for joint in spec.joints:
        if joint.name == "base_joint":
            continue
        spec.add_actuator(name=joint.name)

    xml = fix_actuators(spec.to_xml())
    xml = add_sensors(xml)
    xml = change_geom_types(xml)
    xml = fix_assets(xml)

    with open(consts.ROOT_PATH / "crab.xml", "w") as f:
        f.write(xml)


if __name__ == "__main__":
    make_crab_xml()
