
import sys
sys.path.append("..")
import numpy as np
from utils import mod_cr, helpers
from node_definition import Node
import matplotlib.pyplot as plt

import pandas as pd

def load_and_extract(filename):
    # Load the CSV file
    df = pd.read_csv(filename, header=None)
    return df.to_numpy()


workspaces_dir = "../workspaces/"
def main():
    w_name = 'workspace_3'
    w_name = workspaces_dir + 'workspace_3CPP'
    print(w_name)
    workspace = helpers.load_object(w_name)

    robot1 = mod_cr.Robot(6e-10, 30) #setting dimensions of the robot

    # calculating initial configuration
    config_init = Node(robot1, 0.001, 0.001, "KINEMATIC_MATLAB") 
    config_init.set_init_guess(np.array([1]*robot1.nd))
    config_init.T = np.eye(4)

    counter = config_init.run_forward_model(workspace, True, "KINEMATIC_CPP")
    config_init.plot_configuration(workspace)
    #plt.savefig('initial_config.png')
    plt.show()
    

if __name__ == "__main__":
    main()
