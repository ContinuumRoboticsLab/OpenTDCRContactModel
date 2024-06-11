import numpy as np
import matplotlib.pyplot as plt
from utils.obstacle_definition import Circle
from utils import mod_cr
from utils.node_definition import Node
#import queue
from utils.helpers import PriorityNodeQueue
import numpy as np
import os

def add_list(list1,list2):
    return list(np.round(np.array(list1)+np.array(list2),4))

class TaskSpace:
    def __init__(self):
        self.width = [-0.1, 0]
        self.height = [0, 0.1]
        self.target = [0,0,0]
        self.target_radius = 0.01
        self.target_orientation = 0

    def set_target(self, target, target_radius, target_orientation):
        self.target = target
        self.target_radius = target_radius
        self.target_orientation = target_orientation

    def set_height(self, height):
        if (height[1] == height[0]):
            raise ValueError("limits equal to each other")
        if (height[1] < height[0]):
            self.height = [height[1], height[0]]
        else:
            self.height = height

    def set_width(self, width):
        if (width[1] == width[0]):
            raise ValueError("limits equal to each other")
        if (width[1] < width[0]):
            self.width = [width[1], width[0]]
        else:
            self.width = width

    def plot_workspace(self, goal=True):
        for i in self.obstacle:
            i.plot()
            plt.minorticks_on()
            ax = plt.gca()
            if goal:
                plt.plot(self.target[0],self.target[2],color= 'firebrick',marker='o')
                c = plt.Circle((self.target[0],self.target[2]), self.target_radius, facecolor='salmon',alpha=0.1, edgecolor='none')
                ax.add_patch(c)
                plt.arrow(self.target[0], self.target[2], 0.02*np.cos(self.target_orientation), 0.02*np.sin(self.target_orientation), color = 'firebrick', head_width = 0.004, alpha = 0.7 )
        mod_cr.plotSettings()
        plt.gca().set_aspect('equal', adjustable='box')
        plt.plot(self.target[0], self.target[2], "ro")
            

    def set_obstacles(self):
        raise NotImplementedError("Subclasses must implement this method")

    def matlab_conversion(self):
        raise NotImplementedError

class TaskspaceCircle(TaskSpace):
    def __init__(self):
        super().__init__()
        self.obstacle = []
        self.ob_center = []
        self.ob_ab = []
        self.abM = []
        self.ob_centerM = [] 
        

    def set_obstacles(self, ellipse : Circle, num_copies, delta):
        """
        Sets up obstacles in the task space.

        This function creates multiple copies of a given SuperEllipse object and adds them to the task space as obstacles.
        The position of each copy is offset in the z-direction by a multiple of the given delta.

        Parameters:
        ellipse (SuperEllipse): The SuperEllipse object to be copied and added as an obstacle.
        num_copies (int): The number of copies of the SuperEllipse object to be created and added.
        delta (float): The distance in the z-direction between each copy of the SuperEllipse object.

        Returns:
        None
        """
        for i in range(num_copies):
            self.obstacle += [Circle(ellipse.ab[0], add_list(ellipse.p,[0,0,i*delta]))]
            self.ob_center += [self.obstacle[-1].p]
            self.ob_ab += [self.obstacle[-1].ab]

    def matlab_conversion(self):
        self.abM = mod_cr.matlab_convertor(self.ob_ab)
        self.ob_centerM = mod_cr.matlab_convertor(self.ob_center)
        
    def cpp_conversion(self):
        ob_center_np=np.array(self.ob_center)
        ab_np=np.array(self.ob_ab)
        self.ob_centerCpp = np.concatenate([ob_center_np,np.ones((ob_center_np.shape[0],1))],1).T
        ab_2d = ab_np[:,:2]
        self.ab = np.stack([ab_2d[:,0], np.ones(ab_2d.shape[0]), ab_2d[:,1], np.ones(ab_2d.shape[0])])
        self.s = ab_np[:,2]


    def distance_to_target(self, node, target):
        position = np.round(node.ee, 5)
        hypotenuse = target - position
        distance = np.round(np.sqrt(np.sum(np.square(hypotenuse))), 5)
        return distance
        
        
    def next_path_heuristic(self, node, target):
        """
        Change this function if you want to try out different heuristics for search.
        This function just takes in a list of next possible paths(nodes) to travel too
        and choose the next one for the robot to go too. Currently it just choose the 
        next node that is closest to the target.
        """
        return self.distance_to_target(node, target)
            

    def generate_path(self, start_node: Node, target=[0, 0, 0], target_radius=0.003, max_iter=1500, filename="samplepath.csv"):
        
        # Which neighbours to consider
        del_length = 0.001
        neigh_traverse = [(del_length, 0), (-del_length, 0), (0, del_length), (0, -del_length), (del_length, del_length)]
        
        # Basic variables
        self.target = target
        next = PriorityNodeQueue()
        next.put((self.next_path_heuristic(start_node, target), start_node))
        parents = {start_node: None}
        node_on_target = None

        counter = 0
        # Main search loop
        while not next.isEmpty():
            priority, curr_node = next.get()
            distance = priority
            print(f"{counter} - Current node heuristic (distance): {priority}, distance: {np.round(curr_node.ee, 5)}")
            
            # Break condition, if tip reached target
            if distance <= target_radius:
                print(f"Goal found at {target}")
                node_on_target = curr_node
                break
            
            # Max iterations break
            if counter >= max_iter:
                print(f"Goal has not been found in {max_iter}, quitting")
                return None
            
            # Generate list of neighbours to consider
            neighbour_list = []
            for j in neigh_traverse:
                length = curr_node.l + j[0]
                tendon = curr_node.l_tendon + j[1]
                if length > 0 and tendon > 0:
                    neighbour_list.append(Node(curr_node.robotobj, length, tendon))
            
            # Find add neighbours to queue, calculate their Foward kinematics
            for neigh in neighbour_list:
                neigh.set_init_guess(curr_node.var[0,::3])
                exitflag = neigh.run_forward_model(self, True, "KINEMATIC_CPP")
                if exitflag:
                    pass
                else:
                    print(neigh.l, neigh.l_tendon, "Model computation failed")
                
                if neigh not in parents:
                    priority = self.next_path_heuristic(neigh, target)
                    # import pdb; pdb.set_trace()
                    next.put((priority, neigh))
                    parents[neigh] = curr_node
            counter += 1
        
        # Write joint space path to file
        tmp_filename = f"{filename}tmp"
        with open(tmp_filename, "w+") as f:
            while True:
                values = node_on_target.l, node_on_target.l_tendon
                data = f"{round(values[0], 3)}, {round(values[1], 3)}\n"
                f.write(data)
                node_on_target = parents[node_on_target]
                
                if node_on_target is None:
                    break
        
        # Reverse file order, since we wrote it from last node to first
        with open(tmp_filename, "r") as f:
            with open(filename, "w+") as f2:
                lines = reversed(f.readlines())
                for l in lines:
                    f2.write(l)
        print(f'Written path too: {filename}')
        os.remove(tmp_filename)

    
        