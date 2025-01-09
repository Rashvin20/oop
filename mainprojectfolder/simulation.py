import pybullet as p
import pybullet_data
import time
from gripper import Gripper
from object_handler import ObjectHandler
from environment import Environment
from datalogger import DataLogger

class Simulation:
    def __init__(self):
        self.environment = Environment()
        self.gripper = Gripper()
        self.object_handler = ObjectHandler()
        self.data_logger = DataLogger(self.gripper)

    def run(self, num):
        clid = p.connect(p.SHARED_MEMORY)
        if clid < 0:
            p.connect(p.GUI)

        self.environment.setup_environment()
        self.gripper.setup(self.environment)

        for _ in range(num):
            self.gripper.generate_grasp_data(self.object_handler.obj_position)
            p.stepSimulation()
            time.sleep(0.01)
            self.environment.reset_simulation(self.gripper)
            self.object_handler.reset()
        
        p.disconnect()
                
if __name__ == "__main__":
    input = int(input("Enter the number of simulations: "))
    sim = Simulation()
    sim.run(input)
