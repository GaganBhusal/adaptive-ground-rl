import genesis as gs
import torch
from src.utils import JointController
import numpy as np


gs.init(backend=gs.gpu, logging_level = "info")

scene = gs.Scene(show_viewer=True)
plane = scene.add_entity(gs.morphs.Plane())
go = scene.add_entity(
    gs.morphs.URDF(file='/home/yayy/My/Codeeeeee/Simulators/Genesis/genesis/assets/urdf/go2/urdf/go2.urdf'),
)

scene.build()


joints_local_idx = []
ranges = []
for links in go.links:
    joints = links.joints
    for joint in joints:
        joints_local_idx.append(joint.dof_idx_local)
        ranges.append(np.rad2deg(joint.dofs_limit))
        print(f"Joint Name : {joint.name}, DOF : {joint.n_dofs}, type : {type(joint)}, Axis : {torch.rad2deg(torch.tensor(joint.dofs_limit))}")


total_joints = 12

# tensor([[-60.0001,  60.0001]], dtype=torch.float64)
# tensor([[-60.0001,  60.0001]], dtype=torch.float64)
# tensor([[-60.0001,  60.0001]], dtype=torch.float64)
# tensor([[-60.0001,  60.0001]], dtype=torch.float64)
# tensor([[-90.0002, 200.0024]], dtype=torch.float64)
# tensor([[-90.0002, 200.0024]], dtype=torch.float64)
# tensor([[-30.0001, 260.0025]], dtype=torch.float64)
# tensor([[-30.0001, 260.0025]], dtype=torch.float64)
# tensor([[-155.9992,  -48.0001]], dtype=torch.float64)
# tensor([[-155.9992,  -48.0001]], dtype=torch.float64)
# tensor([[-155.9992,  -48.0001]], dtype=torch.float64)
# tensor([[-155.9992,  -48.0001]], dtype=torch.float64)


print(ranges)


go.set_dofs_kp(
    torch.tensor([100] * 12),
    dofs_idx_local = joints_local_idx[1:]

)

go.set_dofs_kv(
    torch.tensor([10] * 12),
    dofs_idx_local = joints_local_idx[1:]

)


for links in go.links[1:]:
    joints = links.joints
    for joint in joints:
        print(f"Joint name: {joint.name}, local index: {joint.dof_idx_local}")


controller = JointController(ranges[1:], [0, 0, 0, 0, 46, 46, 57, 57, -86, -86, -86, -86])

while True:
    z_coordinate_base = go.get_link("base").get_pos()[2]
    print(z_coordinate_base)

    target_angles= np.deg2rad(controller.read_values())
    go.control_dofs_position(
            target_angles, 
            dofs_idx_local = joints_local_idx[1:]
        )


    scene.step()
    controller.update()




"""
-10 to 10
------
25 to 70
-----
40 to 110
last
-130 to -75
"""