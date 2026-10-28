"""URDF of the Skydio X2 with an OpenMANIPULATOR-X arm mounted upside down below the airframe.

The X2 inertial data are those of the MuJoCo Menagerie model (mass 1.325 kg, with the inertia about
the body center of mass given in its principal frame). The arm data are those of the ROBOTIS
description file. The wrist is fixed and the end effector is a frame at the tip of the gripper.
"""
import os
import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.abspath(os.path.join(here, '..', '..', 'assets'))

X2_MASS = 1.325
X2_COM = (0.0, 0.0, 0.053962264150943406)
X2_RPY = (-1.5707963267948966, -1.4837337126028676, 3.141592653589793)
X2_INERTIA = (0.06071129484278422, 0.036468403270423284, 0.02541169811320754)
X2_MESH_RPY = (1.5707963267948966, 0.0, 3.141592653589793)


def inertial(mass, xyz, rpy, I):
    """I = (ixx, iyy, izz, ixy, ixz, iyz) about the center of mass in the frame given by xyz, rpy"""
    return f"""    <inertial>
      <origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}" rpy="{rpy[0]} {rpy[1]} {rpy[2]}"/>
      <mass value="{mass}"/>
      <inertia ixx="{I[0]}" iyy="{I[1]}" izz="{I[2]}" ixy="{I[3]}" ixz="{I[4]}" iyz="{I[5]}"/>
    </inertial>"""


def make(path=os.path.join(ASSETS, 'aerial_manipulator_drake.urdf')):
    omx = os.path.join(ASSETS, 'openmanipulator_x', 'meshes_obj')
    x2obj = os.path.join(ASSETS, 'skydio_x2', 'assets', 'X2_lowpoly.obj')
    rpy_i, rpy_mesh = X2_RPY, X2_MESH_RPY
    u = f"""<?xml version="1.0"?>
<robot name="aerial_manipulator">
  <material name="dark"><color rgba="0.25 0.25 0.27 1"/></material>
  <material name="red"><color rgba="0.9 0.1 0.1 1"/></material>
  <link name="x2">
{inertial(X2_MASS, X2_COM, rpy_i, (X2_INERTIA[0], X2_INERTIA[1], X2_INERTIA[2], 0, 0, 0))}
    <visual>
      <origin xyz="0 0 0" rpy="{rpy_mesh[0]} {rpy_mesh[1]} {rpy_mesh[2]}"/>
      <geometry><mesh filename="{x2obj}" scale="0.01 0.01 0.01"/></geometry>
    </visual>
  </link>
  <link name="arm_base">
{inertial(7.9119962e-02, (3.0876154e-04, 0, -1.2176461e-04), (0, 0, 0), (1.2505234e-05, 2.1898364e-05, 1.9267361e-05, 0, -1.7855208e-07, 0))}
    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="{omx}/link1.obj"/></geometry><material name="dark"/></visual>
  </link>
  <joint name="mount" type="fixed">
    <parent link="x2"/><child link="arm_base"/>
    <origin xyz="0.02 0 -0.005" rpy="{np.pi} 0 0"/>
  </joint>
  <link name="link2">
{inertial(9.8406837e-02, (-3.0184870e-04, 5.4043684e-04, 0.047433464), (0, 0, 0), (3.4543422e-05, 3.2689329e-05, 1.8850320e-05, -1.6031095e-08, -3.8375155e-07, 2.8511935e-08))}
    <visual><origin xyz="0 0 0.019" rpy="0 0 0"/><geometry><mesh filename="{omx}/link2.obj"/></geometry><material name="dark"/></visual>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="arm_base"/><child link="link2"/>
    <origin xyz="0.012 0 0.017" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.1" upper="3.1" effort="4" velocity="5"/>
  </joint>
  <link name="link3">
{inertial(1.3850917e-01, (1.0308393e-02, 3.7743363e-04, 1.0170197e-01), (0, 0, 0), (3.3055381e-04, 3.4290447e-04, 6.0346498e-05, -9.7940978e-08, -3.8505711e-05, -1.5717516e-06))}
    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="{omx}/link3.obj"/></geometry><material name="dark"/></visual>
  </link>
  <joint name="joint2" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0 0 0.0595" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="4" velocity="5"/>
  </joint>
  <link name="link4">
{inertial(1.3274562e-01, (9.0909590e-02, 3.8929816e-04, 2.2413279e-04), (0, 0, 0), (3.0654178e-05, 2.4230292e-04, 2.5155057e-04, -1.2764155e-06, -2.6874417e-07, 1.1559550e-08))}
    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="{omx}/link4.obj"/></geometry><material name="dark"/></visual>
  </link>
  <joint name="joint3" type="revolute">
    <parent link="link3"/><child link="link4"/>
    <origin xyz="0.024 0 0.128" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="4" velocity="5"/>
  </joint>
  <link name="link5">
{inertial(1.4327573e-01, (4.4206755e-02, 3.6839985e-07, 8.9142216e-03), (0, 0, 0), (8.0870749e-05, 7.5980465e-05, 9.3127351e-05, 0, -1.0157896e-06, 0))}
    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="{omx}/link5.obj"/></geometry><material name="dark"/></visual>
    <visual><origin xyz="0.0817 0.021 0" rpy="0 0 0"/><geometry><mesh filename="{omx}/gripper_left_palm.obj"/></geometry><material name="dark"/></visual>
    <visual><origin xyz="0.0817 -0.021 0" rpy="0 0 0"/><geometry><mesh filename="{omx}/gripper_right_palm.obj"/></geometry><material name="dark"/></visual>
    <visual><origin xyz="0.126 0 0" rpy="0 0 0"/><geometry><sphere radius="0.008"/></geometry><material name="red"/></visual>
  </link>
  <joint name="wrist_fixed" type="fixed">
    <parent link="link4"/><child link="link5"/>
    <origin xyz="0.124 0 0" rpy="0 0 0"/>
  </joint>
  <link name="ee"/>
  <joint name="ee_joint" type="fixed">
    <parent link="link5"/><child link="ee"/>
    <origin xyz="0.126 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""
    open(path, 'w').write(u)
    return path


if __name__ == "__main__":
    print(make())
