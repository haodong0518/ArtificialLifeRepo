import dm_control.mujoco
import mujoco_viewer
import mujoco
import numpy as np
import xml.etree.ElementTree as ET
# tag means <mujoco>,<body>
# attrib means the gravity="0 0 -9.81" , the attribute in the <>
# text can also be stored between angle brakets <>

m = dm_control.mujoco.MjModel.from_xml_path("car.xml")
d = dm_control.mujoco.MjData(m)

viewer = mujoco_viewer.MujocoViewer(m, d)

tree = ET.parse('car.XML')
root = tree.getroot()

motors = m.nu
step = np.array([2, 2, 0, 0])
d.ctrl[:motors] = step

for i in range(10000): 
    if viewer.is_alive:
        d.ctrl[:motors] = step*100 
        mujoco.mj_step(m, d)
        viewer.render()
    else:
        break
# viewer.close()
