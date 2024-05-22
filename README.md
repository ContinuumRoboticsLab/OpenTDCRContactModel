# OpenTDCRContactModel
![alt text](https://crl.utm.utoronto.ca/assets/images/CRLab%20logo_dark_1_large.png)

Learn more about CRL: https://crl.utm.utoronto.ca/


This is an open-source implementation of a 2D kinematic model of a single segment TDCR. 


Current model implementations are based on Ashwin et al.'s work^[1]. 

Additional details on the backbone representation and optimisation setup can be found in^[2]. The model available in this repo has been used to simulate the robot interacting with the environment, and consequently generate motion plans for contact-aided navigation.

[1] K.P. Ashwin, Soumya Kanti Mahapatra, Ashitava Ghosal, Profile and contact force estimation of cable-driven continuum robots in presence of obstacles, _Mechanism and Machine Theory_, Volume 164, 2021

[2] P. Rao, O. Salzman and J. Burgner-Kahrs, "Towards Contact-Aided Motion Planning for Tendon-Driven Continuum Robots," in IEEE Robotics and Automation Letters, vol. 9, no. 5, pp. 4687-4694, May 2024.

## Collab Demo
We have created an interactive Google Collab demo to play with some of the features before installing it on your system. It can be accessed at this link
https://colab.research.google.com/drive/12blCye60rOwlRdw1ya80a9P254LK0z9w?usp=sharing

## Taskspace generation
The 2D plane in which the robot operates is the xz plane. 
The current implementation supports circular obstacles. Non-convex and more complex obstacles can be created by superimposing multiple circular obstacles. 

Each obstacle can be defined by creating an object of the class `SuperEllipse([radius, radius, 2], [x, y,z])`. An object of `Taskspace` class contains multiple such obstacles and can be set by calling `takspace.set_obstacles(circle1, n, z_delta)`, where the function adds `n` number of copies of the input obstacle, offset along the z-direction. 

To create your own taskspace, define a taskspace object and simply add obstacles to it.
```
from utils.taskspace import TaskSpace, TaskSpaceSuperEllipse
from utils.obstacle_definition import Obstacle, SuperEllipse

task = TaskSpaceSuperEllipse()
obstacle_1 = SuperEllipse((0.002, 0.0102, 4), (0.16, 0.0, 0.03))
obstacle_2 = SuperEllipse((0.01, 0.0432, 2), (0.06, 0.0, 0.03))
obstacle_3 = SuperEllipse((0.01, 0.0132, 2), (0.0, 0.0, 0.03))
task.set_obstacles(obstacle_1, 1, 1)
task.set_obstacles(obstacle_2, 1, 1)
task.set_obstacles(obstacle_3, 1, 1)    
```
Or you can use the workspace included in ```/workspaces``` with ```workspace = helpers.load_object(workspace_name)```.
To help with defining more complex ellipses, run ```python ./utils/visualizeEllipse.py``` to visualize different values of the major and minor axis. You can directly input the values on screen into your ```SuperEllipse``` object constructor.




### Details about the robot
The robot can be defined by creating an object of the class `Robot(radius, number_of_disks)` defined in `utils/mod_cr`, woith radius in m and the input number of disks. Since the model uses a piece-wise constant curvature arc representation, the number of disks is used to simply discretize the backbone into that many subsegments.  

A `Node(robot, segment_length, tendon_length)` object is used to identify a configuration of the robot with _tendon length_ and _segment length_, with the robot being able to change both of them. The robot has two degrees of freedom : 1 from length insertion/retraction and 1 from bending due to pulling/releasing the tendon length at [r_disk, 0, 0]. 


### Running the forward model
The forward model can be called as a member function of the `Node` class. Since the model is a complex optimisation routine, with contact constraints, a close initial guess of the curvatures (of size `1xnumber_of_disk`) is required for good convergence of the model. 
Therefore, it is important to set an initial guess so that the resulting model respects the history of motion. See[2] for more details. 

Calling `exitflag = node.set_init_guess(prev_node.var[0,::3])` sets the initial guess as the kappa_x values of the previous nodes curvatures values. 

If the model converges, `exitflag` is returned as `True`. The solved curvature values are available in `node.var`, which is of size `3xnumber_of_disk`, as it stores (kappa_x, kappa_y, tau) for each subsegment. This functionality has been retained for future 3D extension of the model. 

The forward model can be called by `node.run_forward_model(taskspace, bool_flag, "$model_type$")`, where `$model_type$` can either be KINEMATIC_CPP or KINEMATIC_MATLAB. The `bool_flag` is an added functionality that considers a reduced taskspace, where only the obstacles close to the guess provided to the model. 

### Running a sample trajectory
To create a custom pathing for your robot, run
```
 workspace.generate_path(config_init, target=[x, y, z], filename='filename')
```
on your workspace object. This will save a csv file which you can then load in later on for your robot to follow. An example is provided in example.py. Some sample paths are provided in `/sample_paths`. Each .csv file contains a list of `mx2` values, with each row containing [l_segment, l_tendon] that the robot is actuated with. By setting the initial guess of the model to be the previous rows solution, the resulting robot shape for each row can be simulated and plotted. There is added functionality to generate the resulting motion in an .mp4 file. 

## Installation instructions

First clone this repo
```
git clone https://github.com/ContinuumRoboticsLab/OpenTDCRContactModel.git
cd OpenTDCRContactModel
```
Create a conda enviroment. This must be done with sudo as some of the python files
require it.
``` 
sudo $(which conda) env create -f environment.yml --prefix ./envs
```
Activate enviroment
```
conda activate ./envs
```
If you would like to use MATLAB instead of our CPP optimizers, please run ```pip install matlabengine``` make sure MATLAB is installed on your system first.


#### Install system packages
```
sudo apt update
```
```
sudo apt install cmake libopenblas-dev build-essential libnlopt-dev
```

nlopt must be built from source
```
git clone https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install

cd ../../
sudo rm -rf nlopt
```

#### Build the pcca_solver file
```
cd utils_model/cpp
mkdir cmake-build-release
cmake -DCMAKE_BUILD_TYPE=Release -G 'CodeBlocks - Unix Makefiles' -B ./cmake-build-release
cd cmake-build-release
make pcca_solver
cd ../../../
```



### More Information : 
If you found the provided implementation of the TDCR contact model helpful or used parts of it yourself, please refer to it using the following BibTeX entry:

@ARTICLE{Rao2024,
  author={Rao, Priyanka and Salzman, Oren and Burgner-Kahrs, Jessica},
  journal={IEEE Robotics and Automation Letters}, 
  title={Towards Contact-Aided Motion Planning for Tendon-Driven Continuum Robots}, 
  year={2024},
  volume={9},
  number={5},
  pages={4687-4694},
}
## Contributors

Thank you to all the people who have contributed to this project:
- [@priyankarao257](https://github.com/priyankarao257)
- [@FlipperCoin](https://github.com/FlipperCoin)
- [@Nicholas-Baldassini](https://github.com/Nicholas-Baldassini)