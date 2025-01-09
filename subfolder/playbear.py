import pybullet as p
import time
import numpy as np
import pybullet_data
import pandas as pd
import os

#this is not the main project
#this is a simulation of a bear gripper if one want to simulate the grasp using inputs from terminal

class BearGripper:

    def __init__(self):
        self.open = False
        self.robot_model = None
        self.gripperId = None
        self.hand_base_controller = None
        self.numJoints = None
        self.obj_id = None
        self.obj_position = None
#code to grasp the object
    def grasp(self):
        print("Closing the gripper...")
        closed = True
        iteration = 0
        
        
        p.changeConstraint(self.hand_base_controller,jointChildFrameOrientation=p.getQuaternionFromEuler([np.pi, 0, 0]), maxForce=500)
        joints = self.getJointPosition()
        

        while closed and self.open:
            closed = False

            joints = self.getJointPosition()
            for k in range(0, self.numJoints):
                if k == 2 or k == 5 or k == 8:  
                    goal = 0.05
                    if joints[k] >= goal:
                        p.setJointMotorControl2(self.robot_model, k, p.POSITION_CONTROL,
                                                targetPosition=joints[k] - 0.05, maxVelocity=0.5, force=1)
                        closed = True
                elif k == 6 or k == 3 or k == 9:  
                    goal = 0.05
                    if joints[k] <= goal:
                        p.setJointMotorControl2(self.robot_model, k, p.POSITION_CONTROL,
                                                targetPosition=joints[k] - 0.05, maxVelocity=0.5, force=3)
                        closed = True
                elif k == 1 or k == 4 or k == 7:  
                    pos = 0.05
                    if joints[k] <= pos:
                        p.setJointMotorControl2(self.robot_model, k, p.POSITION_CONTROL,
                                                targetPosition=joints[k] + 0.05, maxVelocity=1, force=5)
                        closed = True
            
            iteration += 1
            if iteration > 1000:
                print("Exceeded max iterations!")
                break

            p.stepSimulation()
            time.sleep(0.01)

        self.open = False
        print("Gripper is closed.")
        joints = self.getJointPosition()
        
    #code to open gripper        
    def openGripper(self):
        print("Opening the gripper...")
        closed = True
        iteration = 0

        while closed and not self.open:
            closed = False
            joints = self.getJointPosition()
            for k in range(0, self.numJoints):
                if k == 2 or k == 5 or k == 8:  # Lower finger joints
                    goal = 0.3
                    if joints[k] >= goal:
                        p.setJointMotorControl2(self.robot_model, k, p.POSITION_CONTROL,
                                                targetPosition=joints[k] + 0.05, maxVelocity=1, force=5)
                        closed = True
                elif k == 6 or k == 3 or k == 9:  # Upper finger joints
                    goal = 0.3
                    if joints[k] <= goal:
                        p.setJointMotorControl2(self.robot_model, k, p.POSITION_CONTROL,
                                                targetPosition=joints[k] + 0.05, maxVelocity=1, force=5)
                        closed = True
                elif k == 1 or k == 4 or k == 7:  # Base finger joints
                    pos = 0.3
                    if joints[k] <= pos:
                        p.setJointMotorControl2(self.robot_model, k, p.POSITION_CONTROL,
                                                targetPosition=joints[k] - 0.05, maxVelocity=1, force=5)
                        closed = True
            
            iteration += 1
            if iteration > 500:
                print("Exceeded max iterations!")
                break

            p.stepSimulation()
            time.sleep(0.01)

        self.open = True
        print("Gripper is open.")
        
    def getJointPosition(self):
        joints = []
        for i in range(0, self.numJoints):
            joints.append(p.getJointState(self.robot_model, i)[0])
        return joints
#code to randomly spwan object
    def spawn_object(self, position=None):
        if position is None:
            position = [np.random.uniform(-0.5, 0.5), np.random.uniform(-0.5, 0.5), 0.2]
        orientation = p.getQuaternionFromEuler([np.pi/2, 0, 0])
        
        
        obj_id = p.loadURDF("teddy_vhacd.urdf", position, orientation, globalScaling=4.5, useFixedBase=False)

        p.changeDynamics(obj_id, -1, lateralFriction=1.0, spinningFriction=0.5, rollingFriction=0.2)
        
        for _ in range(100):
            p.stepSimulation()
            time.sleep(0.01)

        aabb_min, aabb_max = p.getAABB(obj_id)


        length_x = aabb_max[0] - aabb_min[0]  
        length_y = aabb_max[1] - aabb_min[1]  
        length_z = aabb_max[2] - aabb_min[2]  


        print(f"Cube Dimensions: X: {length_x}, Y: {length_y}, Z: {length_z}")
        self.obj_id = obj_id
        self.obj_position = position


        
        return obj_id, position
#code to approach top of object
#code is different compared to one in main folder as it first moves to an intermediate position to avoid a sudden movement.
    def move_gripper_to_object(self):
   
        safe_position = [0, 0, 1.5]  
        safe_orientation = p.getQuaternionFromEuler([np.pi, 0, -(np.pi/18)])  
        p.changeConstraint(
            self.hand_base_controller,
            safe_position,
            jointChildFrameOrientation=safe_orientation,
            maxForce=1000
        )
        for _ in range(200): 
            p.stepSimulation()
            time.sleep(0.01)

        
        if not self.open:
            self.openGripper()
        for _ in range(200):  
            p.stepSimulation()
            time.sleep(0.01)

        
        obj_position, _ = p.getBasePositionAndOrientation(self.obj_id)

        
        intermediate_position = [obj_position[0], obj_position[1], obj_position[2] + 1.0]
        new_orientation=p.getQuaternionFromEuler([0,0,np.pi/18])
        p.changeConstraint(
            self.hand_base_controller,
            intermediate_position,
            jointChildFrameOrientation=safe_orientation,
            maxForce=500
        )
        for _ in range(200):
            p.stepSimulation()
            time.sleep(0.01)

        #add offset so that we can get different grasps data to train the classifier
        x_offset = np.random.uniform(-0.01, 0.01)
        y_offset = np.random.uniform(-0.024, 0.024)
        z_offset = np.random.uniform(0.5, 0.6)
        target_position = [obj_position[0]+x_offset, obj_position[1]+y_offset, obj_position[2] + z_offset]
        p.changeConstraint(
            self.hand_base_controller,
            target_position,
            jointChildFrameOrientation=safe_orientation,
            maxForce=500
        )
        for _ in range(300): 
            p.stepSimulation()
            time.sleep(0.01)

        
        p.changeConstraint(
            self.hand_base_controller,
            target_position,
            jointChildFrameOrientation=safe_orientation,
            maxForce=1000
        )
        for _ in range(200):
            p.stepSimulation()
            time.sleep(0.01)
#lift the object
    def lift_object(self):
        if self.obj_position is None:
            print("No object spawned to lift.")
            return

        
        position, orientation = p.getBasePositionAndOrientation(self.gripperId)
        target_position = [position[0], position[1], position[2] + 0.5]  
        target_orientation = orientation

       
        num_steps = 100 
        step_size = 0.005  

        for i in range(num_steps):
            
            intermediate_position = [position[0], position[1], position[2] + i * step_size]
            
            
            p.changeConstraint(self.hand_base_controller, intermediate_position, jointChildFrameOrientation=target_orientation, maxForce=50)

            # Step the simulation forward
            p.stepSimulation()
            time.sleep(0.01)  # Adjust time step if needed

        
        obj_position, _ = p.getBasePositionAndOrientation(self.obj_id)
        gripper_position, _ = p.getBasePositionAndOrientation(self.gripperId)

        
        print(f"Object Position: {obj_position}")
        print(f"Gripper Position: {gripper_position}")
        distance = np.linalg.norm(np.array(gripper_position) - np.array(obj_position))
        print(f"Distance: {distance}")
#distance to determine if its a good grasp or not.
#distance is different as scaling is different.
        if distance < 0.59:
            label="Good Grasp"
            print("Good grasp!")
        else:
            label="Bad Grasp"
            print("Bad grasp!")

        contactpoints = p.getContactPoints(bodyA=self.obj_id, bodyB=self.gripperId)
        contact_pos=None
        contact_norm=None
        contact_force=None

        if contactpoints:
            contact_pos=contactpoints[-1][5]
            contact_norm=contactpoints[-1][7]
            contact_force=contactpoints[-1][9]

        if contact_pos is not None:
            print(f"Final Contact Position: {contact_pos}")
            print(f"Final Contact Normal: {contact_norm}")
            print(f"Final Contact Force: {contact_force}")
        else:
            print("No contact points detected.")

        self.save_results_to_excel(distance, label, contact_force, contact_pos, contact_norm)


    def save_results_to_excel(self,distance, label, force, contact_pos, contact_norm):
        
        data = {
            "Distance": [distance],
            "Contact_Force": [force],
            "Contact_Position_X": [contact_pos[0]],
            "Contact_Position_Y": [contact_pos[1]],
            "Contact_Position_Z": [contact_pos[2]],
            "Contact_Normal_X": [contact_norm[0]],
            "Contact_Normal_Y": [contact_norm[1]],
            "Contact_Normal_Z": [contact_norm[2]],
            "Grasp": [label],
        }

        #code to save the file named after our team and check if it already exists or not.
        file_name = "Team18Results.csv"
        if os.path.exists(file_name):
                        
            existing_data = pd.read_csv(file_name)
            new_data = pd.concat([existing_data, pd.DataFrame(data)], ignore_index=True)
        else:
                        
            new_data = pd.DataFrame(data)

                    
        new_data.to_csv(file_name, index=False)
        print(f"Results saved to {os.path.abspath(file_name)}")      
    def continue_simulation(self):
        
        for _ in range(1000):
            p.stepSimulation()
            time.sleep(0.01)

    def reset(self):
        
        p.resetSimulation()
        
        
        self.setup()
        
        # Reset object and gripper state (if necessary)
        self.obj_id = None 
        self.obj_position = None
        print("Simulation reset completed.")

    def setup(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.loadURDF("plane.urdf")
        path = "./Robots/grippers/threeFingers/sdh/sdh.urdf"
        
        # Set the gripper's initial position and orientation
        initial_position = [0, 0, 0] 
        initial_orientation = p.getQuaternionFromEuler([0, 0, 0])  # Facing downwards

        self.gripperId = p.loadURDF(path, initial_position, initial_orientation,globalScaling=2, useFixedBase=False)
        self.numJoints = p.getNumJoints(self.gripperId)
        self.robot_model = self.gripperId

        # Add constraint to allow movement
        self.hand_base_controller = p.createConstraint(
            parentBodyUniqueId=self.gripperId,
            parentLinkIndex=-1,
            childBodyUniqueId=-1,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=initial_position,
            parentFrameOrientation=initial_orientation,
            childFramePosition=[0, 0, 0],
            childFrameOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
        p.changeConstraint(self.hand_base_controller, initial_position, maxForce=500)
        
        # Add friction to the gripper's links
        for i in range(self.numJoints):
            p.changeDynamics(self.gripperId, i, lateralFriction=1.0, spinningFriction=0.5, rollingFriction=0.2, linearDamping=0.04, angularDamping=0.04)
#run method so as to control the gripper using keyboard inputs.
    def run(self):
        clid = p.connect(p.SHARED_MEMORY)
        if clid < 0:
            p.connect(p.GUI)

        self.setup()

        while True:
            user_input = input("\nEnter a number ('1.spawnObject', '2.gripperApproach', '3.openGripper', '4.closeGripper', '5.liftObject', '6.reset', '7.Step','8.end'): ")

            if user_input == "1":
                self.obj_id, self.obj_position = self.spawn_object()
                print(f"Spawned object at position: {self.obj_position}")

            elif user_input == "2":
                if self.obj_position is None:
                    print("No object spawned yet!")
                    continue
                else:
                    self.move_gripper_to_object()
                    print("Moved gripper to object position.")

            elif user_input == "3":
                if self.open:
                    print("Gripper is already open!")
                    continue
                else:
                    self.openGripper()

            elif user_input == "4":
                if not self.open:
                    print("Gripper is already closed!")
                    continue
                else:
                    self.grasp()
                    print("Gripper closed.")

            elif user_input == "5":
                self.lift_object()

            elif user_input == "6":
                self.reset() 
                print("Simulation has been reset.")
            
            elif user_input == "7":
                self.continue_simulation()
                print("Simulation continued.")

            elif user_input == "8":
                print("Ending the simulation...")
                p.disconnect()
                break

            else:
                print("Invalid command! Please enter a valid number.")
            
if __name__ == "__main__":
    obj = BearGripper()
    obj.run()
