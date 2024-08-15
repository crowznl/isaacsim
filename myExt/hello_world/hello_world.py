# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core import World
import numpy as np
from omni.isaac.core.objects import DynamicCuboid
# Note: checkout the required tutorials at https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html


class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        #world = self.get_world()  # relay to the BaseSample
        world =World.instance()  # retrieve the current instance of the World across files and extensions.
        world.scene.add_default_ground_plane()

        my_cube = world.scene.add(
            DynamicCuboid(
                prim_path="/World/random_cube",
                name="my_cube",  # unique name
                position=np.array([0.0, 0.0, 1.0]),
                orientation=np.array([0.0, 0.0, 0.0]),
                scale=np.array([0.1, 0.1, 0.1]),
                color=np.array([0.0, 0.0, 1.0]),  # RGB channels, range [0, 1]
            )
        )
        return

    async def setup_post_load(self):
        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return
