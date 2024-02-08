import dm_control.mujoco
import mujoco_viewer
import mujoco
import numpy as np
import xml.etree.ElementTree as ET
# tag means <mujoco>,<body>
# attrib means the gravity="0 0 -9.81" , the attribute in the <>
# text can also be stored between angle brakets <>

m = dm_control.mujoco.MjModel.from_xml_path("car.XML")
d = dm_control.mujoco.MjData(m)

viewer = mujoco_viewer.MujocoViewer(m, d)


mj = ET.Element("mujoco")
option = ET.SubElement(mj, "option", gravity = "0 0 -9.81" )
worldbody = ET.SubElement(mj,"worldbody")
light = ET.SubElement(worldbody, "light", diffuse=".5 .5 .5", pos="0 0 3", dir="0 0 -1")
plane = ET.SubElement(worldbody, "geom", type="plane" ,size="5 5 0.1" ,rgba=".9 .9 .9 1")

bodyframe = ET.SubElement(worldbody, "body",name="chassis" ,pos="0 0 0.2" ,euler="0 90 0")
joint = ET.SubElement(bodyframe, "joint",type="free")
geom = ET.SubElement(bodyframe, "geom", type="box", size=".05 .2 .5" ,rgba="0.9 0 0 1")

# wheels
wheel_leftf = ET.SubElement(bodyframe, "body", name="leftf-tire", pos="0 0.3 -0.5" ,euler="90 0 0")
joint_leftf = ET.SubElement(wheel_leftf, "joint", name="leftf-wheel", type="hinge", axis="0 0 -1")
geom_leftf = ET.SubElement(wheel_leftf, "geom",type="cylinder" ,size=".2 0.05" ,rgba="0 0 0 1")

wheel_leftb = ET.SubElement(bodyframe, "body", name="leftb-tire", pos="0 0.3 0.5" ,euler="90 0 0")
joint_leftb = ET.SubElement(wheel_leftb, "joint", name="leftb-wheel", type="hinge", axis="0 0 -1")
geom_leftb = ET.SubElement(wheel_leftb, "geom",type="cylinder" ,size=".2 0.05" ,rgba="0 0 0 1")

wheel_rightf = ET.SubElement(bodyframe, "body", name="rightf-tire", pos="0 -0.3 -0.5" ,euler="90 0 0")
joint_rightf = ET.SubElement(wheel_rightf, "joint", name="rightf-wheel", type="hinge", axis="0 0 -1")
geom_rightf = ET.SubElement(wheel_rightf, "geom", type="cylinder" ,size=".2 0.05" ,rgba="0 0 0 1")

wheel_rightb = ET.SubElement(bodyframe, "body", name="rightb-tire", pos="0 -0.3 0.5" ,euler="90 0 0")
joint_rightb = ET.SubElement(wheel_rightb, "joint", name="rightb-wheel", type="hinge", axis="0 0 -1")
geom_rightb = ET.SubElement(wheel_rightb, "geom",type="cylinder" ,size=".2 0.05" ,rgba="0 0 0 1")

iter_times = 10
sub_wheel1 = ET.SubElement(wheel_leftb, "body", pos ="0 0.2 0" , euler = "0 0 0")
geom1 = ET.SubElement(sub_wheel1, "geom", type = "cylinder", size = ".1 0.05", rgba = "1 0 0 1")

sub_wheel2 = ET.SubElement(wheel_leftb, "body", pos ="0 -0.2 0" , euler = "0 0 0")
geom2 = ET.SubElement(sub_wheel2, "geom", type = "cylinder", size = ".1 0.05", rgba = "1 0 0 1")

sub_wheel3 = ET.SubElement(wheel_leftf, "body", pos ="0 0.2 0" , euler = "0 0 0")
geom3 = ET.SubElement(sub_wheel3, "geom", type = "cylinder", size = ".1 0.05", rgba = "1 0 0 1")

sub_wheel4 = ET.SubElement(wheel_leftf, "body", pos ="0 -0.2 0" , euler = "0 0 0")
geom4 = ET.SubElement(sub_wheel4, "geom", type = "cylinder", size = ".1 0.05", rgba = "1 0 0 1")

sub_wheel5 = ET.SubElement(wheel_rightb, "body", pos ="0 0.2 0" , euler = "0 0 0")
geom5 = ET.SubElement(sub_wheel5, "geom", type = "cylinder", size = ".1 0.05", rgba = "1 0 0 1")

sub_wheel6 = ET.SubElement(wheel_rightb, "body", pos ="0 -0.2 0" , euler = "0 0 0")
geom6 = ET.SubElement(sub_wheel6, "geom", type = "cylinder", size = ".1 0.05", rgba = "1 0 0 1")

sub_wheel7 = ET.SubElement(wheel_rightf, "body", pos ="0 0.2 0" , euler = "0 0 0")
geom7 = ET.SubElement(sub_wheel7, "geom", type = "cylinder", size = ".1 0.05", rgba = "1 0 0 1")

sub_wheel8 = ET.SubElement(wheel_rightf, "body", pos ="0 -0.2 0" , euler = "0 0 0")
geom8 = ET.SubElement(sub_wheel8, "geom", type = "cylinder", size = ".1 0.05", rgba = "1 0 0 1")

sub_wheel9 = ET.SubElement(wheel_leftb, "body", pos ="0.2 0 0" , euler = "0 0 0")
geom9 = ET.SubElement(sub_wheel9, "geom", type = "cylinder", size = ".1 0.05", rgba = "1 0 0 1")

sub_wheel10 = ET.SubElement(wheel_leftb, "body", pos ="-0.2 0 0" , euler = "0 0 0")
geom10 = ET.SubElement(sub_wheel10, "geom", type = "cylinder", size = ".1 0.05", rgba = "1 0 0 1")

sub_wheel11= ET.SubElement(wheel_leftf, "body", pos ="0.2 0 0" , euler = "0 0 0")
geom11 = ET.SubElement(sub_wheel11, "geom", type = "cylinder", size = ".1 0.05", rgba = "1 0 0 1")

sub_wheel12 = ET.SubElement(wheel_leftf, "body", pos ="-0.2 0 0" , euler = "0 0 0")
geom12 = ET.SubElement(sub_wheel12, "geom", type = "cylinder", size = ".1 0.05", rgba = "1 0 0 1")

sub_wheel13 = ET.SubElement(wheel_rightb, "body", pos ="0.2 0 0" , euler = "0 0 0")
geom13 = ET.SubElement(sub_wheel13, "geom", type = "cylinder", size = ".1 0.05", rgba = "1 0 0 1")

sub_wheel14 = ET.SubElement(wheel_rightb, "body", pos ="-0.2 0 0" , euler = "0 0 0")
geom14 = ET.SubElement(sub_wheel14, "geom", type = "cylinder", size = ".1 0.05", rgba = "1 0 0 1")

sub_wheel15 = ET.SubElement(wheel_rightf, "body", pos ="0.2 0 0" , euler = "0 0 0")
geom15 = ET.SubElement(sub_wheel15, "geom", type = "cylinder", size = ".1 0.05", rgba = "1 0 0 1")

sub_wheel16 = ET.SubElement(wheel_rightf, "body", pos ="-0.2 0 0" , euler = "0 0 0")
geom16 = ET.SubElement(sub_wheel16, "geom", type = "cylinder", size = ".1 0.05", rgba = "1 0 0 1")



actuator = ET.SubElement(mj,"actuator")
velocity = ET.SubElement(actuator,"velocity", name="leftf-velocity-servo" ,joint="leftf-wheel" ,kv="100")
velocity = ET.SubElement(actuator,"velocity", name="leftb-velocity-servo" ,joint="leftb-wheel" ,kv="100")
velocity = ET.SubElement(actuator,"velocity", name="rightf-velocity-servo" ,joint="rightf-wheel" ,kv="100")
velocity = ET.SubElement(actuator,"velocity", name="rightb-velocity-servo" ,joint="rightb-wheel" ,kv="100")

tree = ET.ElementTree(mj)
tree.write("output3.xml")


# motors = m.nu
# step = np.array([2, 2, 0, 0])
# d.ctrl[:motors] = step

# for i in range(10000): 
#     if viewer.is_alive:
#         d.ctrl[:motors] = step*100 
#         mujoco.mj_step(m, d)
#         viewer.render()
#     else:
#         break
