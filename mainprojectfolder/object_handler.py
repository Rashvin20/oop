import pybullet as p
import numpy as np
import time

class ObjectHandler:
    def __init__(self):
        self.obj_id = None
        self.obj_position = None

    def spawn_object(self, position=None):
        if position is None:
            position = [np.random.uniform(-0.5, 0.5), np.random.uniform(-0.5, 0.5), 0.2]
        orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.obj_id = p.loadURDF("cube_small.urdf", position, orientation, globalScaling=1.5, useFixedBase=False)
        self.obj_position = position

        p.changeDynamics(self.obj_id, -1, lateralFriction=1.0, spinningFriction=0.5, rollingFriction=0.2)
        for _ in range(100):
            p.stepSimulation()
            time.sleep(0.01)
        
        return self.obj_id

    def reset(self):
        self.obj_id = None
        self.obj_position = None
