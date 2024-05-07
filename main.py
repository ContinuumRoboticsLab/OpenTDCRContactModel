import numpy as np
from utils import mod_cr
from node_definition import Node
from utils import helpers
from utils import helpers
import matplotlib.pyplot as plt

def main():
    w_name = 'workspace_3'
    print(w_name)
    workspace = helpers.load_object(w_name)
    
    robot1 = mod_cr.Robot(6e-3, 30) #setting dimensions of the robot with radius (in m) and number of disks

   # calculating initial configuration
   # A node is used to identify a configuration of the robot with tendon length and segment length, with the robot being able to change both of them
   # The robot has two degrees of freedom : 1 from length insertion and 1 from bending due to change in tendon length at [r_disk, 0, 0]
    config_init = Node(robot1, 0.001, 0.001)
    config_init.set_init_guess(np.array([1]*robot1.nd)) 
    config_init.T = np.eye(4)

    #running a 2D forward model to calculate a configuration 
    # model types = [KINEMATIC_CPP or KINEMATIC_MATLAB]
    # currently only 2D model supported
    counter = config_init.run_forward_model(workspace, True, "KINEMATIC_MATLAB") 
    config_init.plot_configuration(workspace)
    plt.savefig('initial_config.png')



if __name__ == "__main__":
    main()
