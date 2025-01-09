import pybullet as p
import pybullet_data

class Environment:
    def setup_environment(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.loadURDF("plane.urdf")

    def reset_simulation(self, gripper):
        p.resetSimulation()
        self.setup_environment()
        gripper.setup(self)
