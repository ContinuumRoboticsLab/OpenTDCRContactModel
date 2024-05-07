# OpenTDCRContactModel

This is an open-source implementation of a 2D kinematic model of a single segment TDCR. 


Current model implementations are based on Ashwin et al.'s work^[1]. 

Additional details on the backbone representation and optimisation setup can be found in^[2]. The model available in this repo has been used to simulate the robot interacting with the environment, and consequently generate motion plans for contact-aided navigation.

[1] K.P. Ashwin, Soumya Kanti Mahapatra, Ashitava Ghosal, Profile and contact force estimation of cable-driven continuum robots in presence of obstacles, _Mechanism and Machine Theory_, Volume 164, 2021

[2] P. Rao, O. Salzman and J. Burgner-Kahrs, "Towards Contact-Aided Motion Planning for Tendon-Driven Continuum Robots," in IEEE Robotics and Automation Letters, vol. 9, no. 5, pp. 4687-4694, May 2024.

### Taskspace generation
The 2D plane in which the robot operates is the xz plane. 
The current implementation supports circular obstacles. Non-convex and more complex obstacles can be created by superimposing multiple circular obstacles. 

Each obstacle can be defined by creating an object of the class `SuperEllipse([radius, radius, 2], [x, y,z])`. An object of `Taskspace` class contains multiple such obstacles and can be set by calling `takspace.set_obstacles(circle1, n, z_delta)`, where the function adds `n` number of copies of the input obstacle, offset along the z-direction. 



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
Some sample paths are provided in `/sample_paths`. Each .csv file contains a list of `mx2` values, with each row containing [l_segment, l_tendon] that the robot is actuated with. By setting the initial guess of the model to be the previous rows solution, the resulting robot shape for each row can be simulated and plotted. There is added functionality to generate the resulting motion in an .mp4 file. 

### Installation instructions

...


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
