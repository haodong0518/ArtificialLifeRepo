import dm_control.mujoco
import mujoco_viewer
import mujoco
import numpy as np
import xml.etree.ElementTree as ET
# tag means <mujoco>,<body>
# attrib means the gravity="0 0 -9.81" , the attribute in the <>
# text can also be stored between angle brakets <>

mj = ET.Element("mujoco")
option = ET.SubElement(mj, "option", gravity = "0 0 -9.81" )
worldbody = ET.SubElement(mj,"worldbody")
light = ET.SubElement(worldbody, "light", diffuse="0.8 0.8 0.8", pos="0 0 3", dir="0 0 -1")
plane = ET.SubElement(worldbody, "geom", type="plane" ,size="5 5 0.1" ,rgba="255 255 255 0.6")

top = ET.SubElement(worldbody, "body", name = "module-top", pos = "0 0 1", euler = "0 0 90")
joint_base = ET.SubElement(top, "joint", type = "free")
geom1 = ET.SubElement(top, "geom", type = "box",name = "motor-side", size=".1 0.5 0.5", rgba =" 0 0 0 1" )

base = ET.SubElement(top, "body", name = "top_module", pos = "2 0 1")
hsa1 = ET.SubElement(base, "joint", name = "hsa1", type = "slide", axis = "1 0 0")
geom2 = ET.SubElement(base, "geom", type = "box",name = "top-side", size=".1 0.5 0.5", rgba =" 0 0 0 1" )

connector = ET.SubElement(worldbody,"body", name = "connector", pos = "1 0 1")


actuator = ET.SubElement(mj, "actuator")
velocity = ET.SubElement(actuator, "velocity", joint = "hsa1", kv = "100")
tree = ET.ElementTree(mj)
tree.write("output4.xml")

# motors = m.nu
# step = np.array([2, 2, 0, 0])
# d.ctrl[:motors] = step
# The basic mujoco wrapper.

# m = dm_control.mujoco.MjModel.from_xml_path("output4.XML")
# d = dm_control.mujoco.MjData(m)
# viewer = mujoco_viewer.MujocoViewer(m, d)

# from dm_control.mujoco.wrapper.mjbindings import enums
# from dm_control.mujoco.wrapper.mjbindings import mjlib
# import PIL.Image
# import os

# physics = dm_control.mujoco.Physics.from_xml_path("output4.XML")
# scene_option = dm_control.mujoco.wrapper.core.MjvOption()
# scene_option.frame = enums.mjtFrame.mjFRAME_GEOM
# scene_option.flags[enums.mjtVisFlag.mjVIS_JOINT] = True
# pixels = physics.render(scene_option=scene_option)
# PIL.Image.fromarray(pixels)

# for i in range(10000): 
#     if viewer.is_alive:
#         #d.ctrl[:motors] = step*100 
#         #mujoco.mj_step(m, d)
#         viewer.render()
#     else:
#         break