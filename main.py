import numpy as np
from utils import mod_cr, helpers
from utils.node_definition import Node
from utils.taskspace import TaskSpace, TaskSpaceSuperEllipse
from utils.obstacle_definition import Obstacle, SuperEllipse
import matplotlib.pyplot as plt
import sys


import pandas as pd


sys.path.append('./utils')
sys.path.append('./media')

def load_and_extract(filename):
    # Load the CSV file
    df = pd.read_csv(filename, header=None)
    return df.to_numpy()

workspaces_dir = "workspaces/"
def main():
    w_name = 'workspaces/workspace_3CPP'
    print("Workspace name: ",w_name)
    
    
    task = TaskSpaceSuperEllipse()
    obstacle_1 = SuperEllipse((0.002, 0.0102, 4), (0.16, 0.0, 0.03))
    obstacle_2 = SuperEllipse((0.01, 0.0432, 2), (0.06, 0.0, 0.03))
    obstacle_3 = SuperEllipse((0.01, 0.0132, 2), (0.0, 0.0, 0.03))
    task.set_obstacles(obstacle_1, 1, 1)
    task.set_obstacles(obstacle_2, 1, 1)
    task.set_obstacles(obstacle_3, 1, 1)
    
    workspace = helpers.load_object(w_name)
    workspace = task

    robot1 = mod_cr.Robot(6e-3, 30) #setting dimensions of the robot

    # calculating initial configuration
    config_init = Node(robot1, 0.001, 0.001)
    config_init.set_init_guess(np.array([1]*robot1.nd))
    config_init.T = np.eye(4)

    counter = config_init.run_forward_model(workspace, True, "KINEMATIC_CPP")
    config_init.plot_configuration(workspace)
    
    plt.show()
    sys.exit(0)
    helpers.saveFigure()
    
    #plt.show()
 
    #generating motion plan based on a provided sample path
    
    print("Computing path")
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
    print("Saving video")
    helpers.visualizing(traced_path[::-1], workspace, "media/sample", show_video=False)


if __name__ == "__main__":
    main()
