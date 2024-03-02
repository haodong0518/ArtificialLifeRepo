import dm_control.mujoco
import mujoco_viewer
import mujoco
import numpy as np
import xml.etree.ElementTree as ET

import math
import copy
import modular
#--** Define Environment & Plane Textures **---------------------------------------------------------
mj = ET.Element("mujoco")
option = ET.SubElement(mj, "option", gravity = "0 0 -9.81" )
worldbody = ET.SubElement(mj,"worldbody")
light = ET.SubElement(worldbody, "light", diffuse="0.8 0.8 0.8", pos="0 0 3", dir="0 0 -1")
asset = ET.SubElement(mj, "asset")
textures = [
    {"type": "skybox", "builtin": "gradient", "rgb1": ".3 .5 .7", "rgb2": "0 0 0", "width": "512", "height": "512"},
    {"name": "body", "type": "cube", "builtin": "flat", "mark": "cross", "width": "128", "height": "128",
    "rgb1": "0.8 0.6 0.4", "rgb2": "0.8 0.6 0.4", "markrgb": "1 1 1", "random": "0.01"},
    {"name": "grid", "type": "2d", "builtin": "checker", "width": "512", "height": "512", "rgb1": ".1 .2 .3",
    "rgb2": ".2 .3 .4"}
]
materials = [
    {"name": "body", "texture": "body", "texuniform": "true", "rgba": "0.8 0.6 .4 1"},
    {"name": "grid", "texture": "grid", "texrepeat": "1 1", "texuniform": "true", "reflectance": ".2"}
]
# Add texture elements
for tex in textures:
    ET.SubElement(asset, "texture", tex)
# Add material elements
for mat in materials:
    ET.SubElement(asset, "material", mat)
plane = ET.SubElement(worldbody, "geom", size="10 10 .05",type="plane", material="grid",condim="3")   
chassis = ET.SubElement(worldbody,"body", name = "chassis" ,pos = "2 2 2")
chassis_joint = ET.SubElement(chassis, "joint", type = "free")
chassis_geom = ET.SubElement(chassis,"geom", type = "box", size=".1 2 1", rgba = ".9 .9 .9 1")
# inertial = ET.SubElement(chassis, "inertial", mass=1)
#--** Building Units** ------------------------------------------------------------

modular.build_unit(chassis,"module1", "1 -1 1")
modular.build_unit(chassis,"module2", "1 1 1")
tree = ET.ElementTree(mj)
actuator = ET.SubElement(mj, "actuator")
velocity = ET.SubElement(actuator, "velocity", joint = "module1slide_box_joint", kv = "2000")
tree.write("actuator.xml")

#--** Motion Control ** -----------------------------------------------
m = dm_control.mujoco.MjModel.from_xml_path("actuator.XML")
d = dm_control.mujoco.MjData(m)
viewer = mujoco_viewer.MujocoViewer(m, d)

motors = m.nu
step = 5
d.ctrl[:motors] = step
duration_per_direction = 50

pos=[]
n = 500
for i in range(n): 
    if i ==0:
        pos.append(copy.copy(d.body('module1module-top').xpos))
    if i == (n-1):
        pos.append(copy.copy(d.body('module1module-top').xpos))
    current_step = i // duration_per_direction
    if viewer.is_alive:
        if current_step%2 == 0 :
            d.ctrl[:motors] = step*100 
            mujoco.mj_step(m, d)
        else:
            d.ctrl[:motors] = -step*100 
            mujoco.mj_step(m, d)
        viewer.render()
    else:
        break

# fitness function = the Euclidean Distance / time
fitness = math.sqrt((pos[0][0]-pos[1][0])**2 + (pos[0][1]-pos[1][1])**2) / n
print(fitness)