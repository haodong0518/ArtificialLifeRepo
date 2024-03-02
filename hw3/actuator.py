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

##
asset = ET.SubElement(mj, "asset")

# Define textures and materials
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

## body define 
top = ET.SubElement(worldbody, "body", name = "module-top", pos = "0 0 1")
free_plate = ET.SubElement(top, "joint", type = "free")
geom_free_plate = ET.SubElement(top, "geom", type = "box",name = "geom_free_plate", size=".1 1 1", rgba ="0.9 0 0 1" )

slide_box_body = ET.SubElement(top, "body", name = "slide_box", pos = "1 0.3 0.3")
slide_box_joint = ET.SubElement(slide_box_body, "joint", name = "slide_box_joint", type = "slide", axis = "1 0 0",range="-0.5 0.5")
geom_slide_box = ET.SubElement(slide_box_body, "geom", type = "box",name = "slide_box_body", size="0.5 0.1 0.1", rgba =" 0 0 0 1" )

hingex_box_body= ET.SubElement(slide_box_body, "body", name = "hingex_box_body", pos = "1 0 0")
hingex_box_joint = ET.SubElement(hingex_box_body, "joint",name = "hingex_box_joint", type = "hinge" ,axis = "1 0 0",range="-45 45")
geom_hingex_box =ET.SubElement(hingex_box_body, "geom", type = "box", name = "hingex_box_body", size ="0.5 0.1 0.1", rgba = "0.9 0 0 1" )

hingey_box_body = ET.SubElement(hingex_box_body, "body", name = "hingey_box_body", pos = "1 0 0")
hingey_box_joint = ET.SubElement(hingey_box_body, "joint", name = "hingey_box_joint", type = "hinge", axis = "0 1 0",range="-45 45")
geom_hingey_box = ET.SubElement(hingey_box_body, "geom" , type = "box",name = "geom_hingey_box" ,size ="0.5 0.1 0.1", rgba = "0.9 0 0 1" )

hingez_box_body = ET.SubElement(hingey_box_body, "body", name = "hingez_box_body", pos = "1 0 0")
hingez_box_joint = ET.SubElement(hingez_box_body, "joint", name = "hingez_box_joint", type = "hinge", axis = "0 0 1",range="-45 45")
geom_hingez_box = ET.SubElement(hingez_box_body, "geom" , type = "box", name = "geom_hingez_box" ,size ="0.5 0.1 0.1", rgba = "0.9 0 0 1" )

slide_plate = ET.SubElement(hingez_box_body, "body", name= "slide_plate", pos = "1 -0.3 -0.3")
slide_plate_joint = ET.SubElement(slide_plate, "joint", name = "slide_plate_joint", type = "slide" , range="-0.5 0.5")
geom_slide_plate = ET.SubElement(slide_plate, "geom", type = "box", name = "slide_plate", size ="0.1 1 1", rgba = "0 0 0 1")

actuator = ET.SubElement(mj, "actuator")
velocity = ET.SubElement(actuator, "velocity", joint = "slide_box_joint", kv = "1000")
tree = ET.ElementTree(mj)
tree.write("actuator.xml")
