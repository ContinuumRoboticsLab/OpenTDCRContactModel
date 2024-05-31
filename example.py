import numpy as np
from utils import mod_cr, helpers
from utils.node_definition import Node
from utils.obstacle_definition import Circle
from utils.taskspace import TaskspaceCircle
import matplotlib.pyplot as plt
import sys


import pandas as pd


sys.path.append('./utils')
sys.path.append('./media')
sys.path.append('./sample_paths')

def load_and_extract(filename):
    # Load the CSV file
    df = pd.read_csv(filename, header=None)
    return df.to_numpy()


def main():    
    workspace = helpers.load_object("workspaces/basicWorkspace")
    robot1 = mod_cr.Robot(6e-3, 30) #setting dimensions of the robot



    # calculating initial configuration
    config_init = Node(robot1, 0.001, 0.001)
    config_init.set_init_guess(np.array([1]*robot1.nd))
    config_init.T = np.eye(4)

    config_init.run_forward_model(workspace, True, "KINEMATIC_CPP")
    workspace.generate_path(config_init, target=[0.064, 0, 0.088], filename='sample_paths/topright.csv')

    config_init.plot_configuration(workspace)
    helpers.saveFigure()
    #workspace.plot_workspace()
    #sys.exit()
    
    # Generate path file
    #workspace.generate_path(config_init, target=[0.06, 0, 0.09], filename='sample_paths/topright.csv')
    
 
    #generating motion plan based on a provided sample path
    print("Computing path")
    prev_guess = config_init.var[0,::3]
    #sample_path = load_and_extract('sample_paths/3_sample_path.csv')
    sample_path = load_and_extract('sample_paths/topright.csv')
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
    print("Saving video")
    helpers.visualizing(traced_path, workspace, "media/sample", show_video=True)


if __name__ == "__main__":
    main()
