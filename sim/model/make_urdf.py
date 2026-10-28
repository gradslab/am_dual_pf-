"""Write a URDF with primitive geometry that matches params.py exactly (used for the Drake
cross-check of the symbolic model; the visual model with real meshes is built separately)."""
import os
from params import default_params


def make_urdf(p, path):
    def inertial(m, com, J):
        return f"""    <inertial>
      <origin xyz="{com[0]} {com[1]} {com[2]}" rpy="0 0 0"/>
      <mass value="{m}"/>
      <inertia ixx="{J[0]}" ixy="0" ixz="0" iyy="{J[1]}" iyz="0" izz="{J[2]}"/>
    </inertial>"""

    def cyl_visual(L, r, z0, color):
        return f"""    <visual>
      <origin xyz="0 0 {z0}" rpy="0 0 0"/>
      <geometry><cylinder radius="{r}" length="{L}"/></geometry>
      <material name="{color}"/>
    </visual>"""

    u = f"""<?xml version="1.0"?>
<robot name="aerial_manipulator">
  <material name="grey"><color rgba="0.3 0.3 0.3 1"/></material>
  <material name="orange"><color rgba="0.9 0.5 0.1 1"/></material>
  <material name="blue"><color rgba="0.2 0.3 0.8 1"/></material>
  <link name="base">
{inertial(p['mq'], (0, 0, 0), (p['Jqx'], p['Jqy'], p['Jqz']))}
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.12 0.12 0.04"/></geometry>
      <material name="grey"/>
    </visual>
  </link>
  <link name="link1">
{inertial(p['m1'], (0, 0, -p['l1'] / 2), (p['J1x'], p['J1y'], p['J1z']))}
{cyl_visual(p['l1'], 0.03, -p['l1'] / 2, 'blue')}
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base"/><child link="link1"/>
    <origin xyz="0 0 {-p['dm']}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="10"/>
  </joint>
  <link name="link2">
{inertial(p['m2'], (0, 0, -p['l2'] / 2), (p['J2x'], p['J2y'], p['J2z']))}
{cyl_visual(p['l2'], 0.015, -p['l2'] / 2, 'orange')}
  </link>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 {-p['l1']}" rpy="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="10"/>
  </joint>
  <link name="link3">
{inertial(p['m3'], (0, 0, -p['l3'] / 2), (p['J3x'], p['J3y'], p['J3z']))}
{cyl_visual(p['l3'], 0.015, -p['l3'] / 2, 'orange')}
  </link>
  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0 0 {-p['l2']}" rpy="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="10"/>
  </joint>
  <link name="ee"/>
  <joint name="ee_joint" type="fixed">
    <parent link="link3"/><child link="ee"/>
    <origin xyz="0 0 {-p['l3']}" rpy="0 0 0"/>
  </joint>
</robot>
"""
    open(path, "w").write(u)
    return path


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    print(make_urdf(default_params(), os.path.join(here, "am_primitive.urdf")))
