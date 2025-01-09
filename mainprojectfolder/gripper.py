import pybullet as p
import time
import numpy as np
from datalogger import DataLogger
from object_handler import ObjectHandler

class Gripper(DataLogger):
    def __init__(self):
        super().__init__(gripper=self)
        self.open = False
        self.robot_model = None
        self.gripperId = None
        self.obj_id = None
        self.hand_base_controller = None
        self.numJoints = None

    def setup(self, environment):
        path = "./Robots/grippers/threeFingers/sdh/sdh.urdf"
        
        # Set the gripper's initial position and orientation
        initial_position = [0, 0, 0]  # Floating at 3 meters above the ground
        initial_orientation = p.getQuaternionFromEuler([0, 0, 0])  # Facing downwards

        self.gripperId = p.loadURDF(path, initial_position, initial_orientation, useFixedBase=False)
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

    def openGripper(self):
        print("Opening the gripper...")
        if self.open:
            print("Gripper is already open.")
            return None
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
                                                targetPosition=joints[k] - 0.05, maxVelocity=1, force=5)
                        closed = True
                elif k == 6 or k == 3 or k == 9:  # Upper finger joints
                    goal = 0.3
                    if joints[k] <= goal:
                        p.setJointMotorControl2(self.robot_model, k, p.POSITION_CONTROL,
                                                targetPosition=joints[k] - 0.05, maxVelocity=1, force=5)
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
        
    def move_gripper_to_object(self):
        
        obj_id = ObjectHandler.spawn_object(self)

        random_x = np.random.uniform(-0.06, 0.06)
        random_y = np.random.uniform(-0.06, 0.06)
        random_z = np.random.uniform(-0.02, 0.02)
        
        # move gripper to air facing downwards
        p.changeConstraint(self.hand_base_controller, [0, 0, 0.5], jointChildFrameOrientation=p.getQuaternionFromEuler([np.pi, 0, 0]), maxForce=500)
        for _ in range(200):
            p.stepSimulation()
            time.sleep(0.01)
            
        self.openGripper()
            
        # Get current gripper and object positions
        obj_position, _ = p.getBasePositionAndOrientation(obj_id)
        
        # Move the gripper above the object
        target_position = [obj_position[0] + random_x, obj_position[1] + random_y, obj_position[2] + 0.5]
        target_orientation = p.getQuaternionFromEuler([np.pi, 0, 0])
        
        # Move the gripper directly to the target position and orientation
        p.changeConstraint(self.hand_base_controller, target_position, jointChildFrameOrientation=target_orientation, maxForce=100)
        
        # Step simulation to smoothly approach the position
        for _ in range(200):
            p.stepSimulation()
            time.sleep(0.01)
            
        target_position = [obj_position[0] + random_x, obj_position[1] + random_y, obj_position[2] + 0.18 + random_z]
        target_orientation = p.getQuaternionFromEuler([np.pi, 0, 0])
        
        # Move the gripper directly to the target position and orientation
        p.changeConstraint(self.hand_base_controller, target_position, jointChildFrameOrientation=target_orientation, maxForce=100)
        
        # Step simulation to smoothly approach the position
        for _ in range(200):
            p.stepSimulation()
            time.sleep(0.01)
        
        p.changeConstraint(self.hand_base_controller, target_position, jointChildFrameOrientation=target_orientation, maxForce=500)
    
    def grasp(self):
        print("Closing the gripper...")
        if not self.open:
            print("Gripper is already closed.")
            return None
        closed = True
        iteration = 0
        p.changeConstraint(self.hand_base_controller, jointChildFrameOrientation=p.getQuaternionFromEuler([np.pi, 0, 0]), maxForce=500)

        while closed and self.open:
            closed = False
            joints = self.getJointPosition()
            for k in range(0, self.numJoints):
                if k == 2 or k == 5 or k == 8:  # Lower finger joints
                    goal = 0.05
                    if joints[k] <= goal:
                        p.setJointMotorControl2(self.robot_model, k, p.POSITION_CONTROL,
                                                targetPosition=joints[k] + 0.05, maxVelocity=0.5, force=2)
                        closed = True
                elif k == 6 or k == 3 or k == 9:  # Upper finger joints
                    goal = 0.05
                    if joints[k] <= goal:
                        p.setJointMotorControl2(self.robot_model, k, p.POSITION_CONTROL,
                                                targetPosition=joints[k] + 0.05, maxVelocity=0.5, force=3)
                        closed = True
                elif k == 1 or k == 4 or k == 7:  # Base finger joints
                    pos = 0.05
                    if joints[k] <= pos:
                        p.setJointMotorControl2(self.robot_model, k, p.POSITION_CONTROL,
                                                targetPosition=joints[k] + 0.05, maxVelocity=1, force=5)
                        closed = True

            iteration += 1
            if iteration > 800:
                print("Exceeded max iterations!")
                break
            
            p.stepSimulation()
            time.sleep(0.01)

        self.open = False
        print("Gripper is closed.")


    def lift_object(self):
        if self.obj_position is None:
            print("No object spawned to lift.")
            return None
        
        # On gravity to check grasp quality
        p.setGravity(0, 0, -9.8)

        # Set a target position slightly above the object
        position, orientation = p.getBasePositionAndOrientation(self.gripperId)
        target_position = [position[0], position[1], position[2] + 0.5]
        target_orientation = orientation

        # Apply the constraint change with increased force
        p.changeConstraint(self.hand_base_controller, target_position, jointChildFrameOrientation=target_orientation, maxForce=50)
        for _ in range(1000):
            p.stepSimulation()
            time.sleep(0.01)  # Adjust time step if needed
    
    def generate_grasp_data(self, contact_points):

        self.move_gripper_to_object()
        
        self.grasp()
        # Check for contact points
        contact_points = p.getContactPoints(self.obj_id, self.gripperId)
        contactArea = self.calculate_contact_area(contact_points)
        
        # Check object position before moving
        obj_position, _ = p.getBasePositionAndOrientation(self.obj_id)
        gripper_position, _ = p.getBasePositionAndOrientation(self.gripperId)
        distance_to_object = np.linalg.norm(np.array(gripper_position) - np.array(obj_position))
        
        # xyz coordinates of the gripper relative to the object
        x = gripper_position[0] - obj_position[0]
        y = gripper_position[1] - obj_position[1]
        z = gripper_position[2] - obj_position[2]
        
        self.lift_object()
    
        # Check object position after moving
        obj_position, _ = p.getBasePositionAndOrientation(self.obj_id)
        gripper_position, _ = p.getBasePositionAndOrientation(self.gripperId)

        # Check if object is close to gripper's position
        distance = np.linalg.norm(np.array(gripper_position) - np.array(obj_position))
        if distance < 0.3:
            print("Good grasp!")
            grasp_quality = 1
        else:
            print("Bad grasp!")
            grasp_quality = 0
        
        # Log grasp data
        self.log_data(contact_points, grasp_quality, gripper_position, obj_position, distance_to_object, contactArea, x, y, z)
        
        
    def getJointPosition(self):
        joints = []
        for i in range(0, self.numJoints):
            joints.append(p.getJointState(self.robot_model, i)[0])
        return joints
