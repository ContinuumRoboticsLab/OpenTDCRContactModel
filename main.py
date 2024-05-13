import numpy as np
from utils import mod_cr, helpers
from node_definition import Node
import matplotlib.pyplot as plt

import pandas as pd

def load_and_extract(filename):
    # Load the CSV file
    df = pd.read_csv(filename, header=None)
    return df.to_numpy()

workspaces_dir = "workspaces/"
def main():
    w_name = 'workspace_3'
    w_name = workspaces_dir + "workspace_3CPP"
    print(w_name)
    workspace = helpers.load_object(w_name)
    
    robot1 = mod_cr.Robot(6e-3, 30) #setting dimensions of the robot with radius (in m) and number of disks

    # calculating initial configuration

    config_init = Node(robot1, 0.001, 0.001)
    config_init.set_init_guess(np.array([1]*robot1.nd)) 

    #running a 2D forward model to calculate a configuration 
   

    counter = config_init.run_forward_model(workspace, True, "KINEMATIC_CPP") 
    config_init.plot_configuration(workspace)
    plt.savefig('initial_config.png')
    

    #generating motion plan based on a provided sample path
    prev_guess = config_init.var[0,::3]
    sample_path = load_and_extract('sample_paths/3_sample_path.csv')
    traced_path = [config_init]*len(sample_path)
    for idx, iter in enumerate(sample_path):
        curr_node = Node(robot1, iter[0], iter[1])
        curr_node.set_init_guess(prev_guess)
        model_exitflag = curr_node.run_forward_model(workspace, True, "KINEMATIC_CPP")
        if model_exitflag:
            prev_guess = curr_node.var[0,::3]
            traced_path[idx] = curr_node
        else:
            print("model did not converge - investigate initial guess / input actuations, at index = ", iter)
        break
    plt.show()


if __name__ == "__main__":
    main()
