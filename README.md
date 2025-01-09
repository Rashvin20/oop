TEAM 18 PROJECT

Description:

This project simulation a robotic gripper grasping an object in a Pybullet-based environment.It features an object spawning at a random location, a gripper gripping it and lifting it. It then logs the grasps data and evaluate whether it is a good grasp or bad grasp.

Note:
The code required for this project are in the 'mainprojectfolder' or you can download the cube_gripper.zip file and run the simulation.py file.

Installation:

python -m env robsim

source robsim/bin/activate

pip install -r requirements.txt


How to run the project:
1. Follow installation instructions.
2. Download the files in main folder in that environment.
3. Run the simulation.py file.(main script file)
4. Observe the simulation in the PyBullet GUI and automatically saves the data to grasp.csv.

Information on Folders and Files:

mainprojectfolder:

1.The dataset in the main folder correlates with the first dataset exlpained in the report.

2.It contains the the training classifiers (grasp_classifier.ipynb) used on grasp_data.csv.

3.The code is structured into different files and are connected to each other.

4.Simulation.py is the main script file.

subfolder:
1. It contains the second dataset mentionned in the report.
2. It contains the training classifiers (trainingrasp.ipynb) used on the second dataset GraspResults.csv.
3. To control the gripper from keyboard inputs in the terminal, execute playbear.py
4. It simulates the gripper grasping a bear (second object).

If you encounter any problems related to modules or files paths, you may need to modify the paths as follows:

Option 1:

(a)  'mainproject' folder:

(i)Error about loading plane:

1.Download the files in the 'planeurdf' folder.

2.Copy and replace the path in line 8 in mainprojectfolder/environment.py with the new path to the plane.urdf downloaded.

(ii)Error about loading gripper:

1.Download the files in the 'threefingers' folder.

2.Copy and replace the path in line 18 of mainprojectfolder/gripper.py with the new path to the sdh.urdf file in the new folder downloaded.

(iii)Error about loading cube:

1.Download the files mainprojectfolder/cube.obj & mainprojectfolder/cube.urdf & mainprojectfolder/cube_small.urdf.

2.Copy and replace the path in line 14 of mainprojectfolder/object_handler.py with the path to cube_small.urdf.

(b) 'subproject' folder:

(i)Error about loading plane:

1.Download the files in the 'planeurdf' folder.

2.Copy and replace the path in line 319 in project_group_18/subfolder/playbear.py with the path to the  plane.urdf downloaded.

(ii)Error about loading gripper:

1.Download the files in the 'threefingers' folder.

2.Copy and replace the path in line 320 of project_group_18/subfolder/playbear.py with the new path to the sdh.urdf file in the new folder downloaded.

(iii)Error about loading bear:

1.Download the files subfolder/teddy_vhacd.urdf & subfolder/teddy2_VHACD_CHs.obj.

2.Copy and replace the path in line 120 of  project_group_18/subfolder/playbear.py with the path to the sdh.urdf file.


NOTE:Ensure all these files downloaded are  within the folder of the file you want to run.

Option 2:

Download RobotSimulator_v1 and run the desired project within this folder.





Contact:
zcabrr0@ucl.ac.uk for more info.




