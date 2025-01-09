import pybullet as p
import pandas as pd
import numpy as np
from scipy.spatial import ConvexHull
import time

class DataLogger:
    def __init__(self, gripper):
        self.data = []  # List to store rows of data (values only)

    def calculate_contact_area(self, contact_points):
        if len(contact_points) <= 2:
            return 0
        positions = [point[5] for point in contact_points]
        positions_np = np.array(positions)
        hull = ConvexHull(positions_np[:, :2])
        return hull.area

    def log_data(self, contact_points, grasp_quality, gripper_position, obj_position, distance_to_object, contact_area, x, y, z):
        dataset = pd.read_csv("grasp_data.csv")
        num_contact_points = int(len(contact_points))  # Ensure conversion to int
        normal_force = float(sum([point[9] for point in contact_points]))  # Convert to float

        # Append a list (row of values) to self.data
        row = [
            contact_area,
            num_contact_points,
            normal_force,
            distance_to_object,
            x,
            y,
            z,
            int(grasp_quality)  # Convert to int
        ]
        # Append the row to the data list
        dataset = pd.concat([dataset, pd.DataFrame([row], columns=dataset.columns)], ignore_index=True)
        dataset.to_csv("grasp_data.csv", index=False)
        print("Data saved to grasp_data.csv.")
        return dataset


