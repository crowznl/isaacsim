# filename: add_object.py
import numpy as np
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core import World

# Instantiate World class
world = World()
world.scene.add_default_ground_plane()

# Add cuboid to the stage
target_cuboid = DynamicCuboid(
    prim_path="/World/Target",  # Path in the stage
    name="target_cuboid",  # Nickname in the scene
    position=np.array([0.6, 0.6, 0.1]),  # position in the world frame of the prim. shape is (3, ).
    # translation in the local frame of the prim(with respect to its parent prim). shape is (3, ).
    # [Error] You cannot define translation and position at the same time
    orientation=np.array([1, 0, 0, 0]),  # Initial orientation as an array [qw, qx, qy, qz]
    color=np.array([1, 0, 0]),  # Normalized RGB color (red)
    size=0.05,  # Size of the cuboid, size*scale = [length, width, height]
    scale=np.array([1, 1, 1]),  # Scale factors for the cuboid
    mass=0.05,  # Mass of the cuboid in kilograms
)

# Register the cuboid in the scene
world.scene.add(target_cuboid)
# Reset the world to reinitialize objects in the scene
world.reset()
# ---------------------------------------------------------------------------------------------
# filename: get_object_info.py

# from omni.isaac.core import World
# # Instantiate World class
# world = World()

# Get the DynamicCuboid instance from the scene
cube = world.scene.get_object("target_cuboid")
print(cube)
# Get the world pose of the object
position, orientation = cube.get_world_pose()
print(f"Position(x,y,z)={position}")
print(f"Orientation(qw,qx,qy,qz)={orientation}")
# Get the linear and angular velocities of the object
# DynamicCuboid is subclass of RigidPrim, which has the dynamic related methods
linear_velocity = cube.get_linear_velocity()
angular_velocity = cube.get_angular_velocity()
mass = cube.get_mass()

print(f"Linear velocity={linear_velocity}")
print(f"Angular velocity={angular_velocity}")
print(f"Mass={mass}")

#---------------------------------------------------------------------------------------
# filename: import_robot.py

from omni.isaac.core.utils.nucleus import get_assets_root_path
# Get isaac sim assets folder root path
# It should be "omniverse://localhost/NVIDIA/Assets/Isaac/4.0"
assets_root_path = get_assets_root_path()
print(assets_root_path)
if assets_root_path is None:
    print("Could not find nucleus server with '/Isaac' folder")

# Get franka in isaac sim official assets folder
robot_asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
print(robot_asset_path)

# Add robot asset reference to stage
# This will create a new XFormPrim and point it to the usd file as a reference
from omni.isaac.core.utils.stage import add_reference_to_stage

add_reference_to_stage(usd_path=robot_asset_path, prim_path="/World/MyRobot")
# ----------------------------------------------------------------------------------------------
# filename: add_robot.py
# Wrap the root of robot prim under a Robot(Articulation) class
# to use high level api to set/ get attributes as well as initializing physics handles needed
from omni.isaac.core.robots import Robot
# from omni.isaac.core.world import World
# world = World()
robot = Robot(prim_path="/World/MyRobot", name="my_robot")
# Add robot to the scene
world.scene.add(robot)
print(robot)
# Resetting the world needs to be called before querying anything related to an articulation specifically.
# Its recommended to always do a reset after adding your assets, for physics handles to be propagated properly
world.reset()

# world.clear()
