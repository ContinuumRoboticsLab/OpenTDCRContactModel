import numpy as np
import matplotlib.pyplot as plt
from utils.obstacle_definition import Circle
from utils import mod_cr
from utils.node_definition import Node
import queue
import numpy as np

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


    def next_path_heuristic(self, neigh_nodes, target):
        """
        Change this function if you want to try out different heuristics for search.
        This function just takes in a list of next possible paths(nodes) to travel too
        and choose the next one for the robot to go too. Currently it just choose the 
        next node that is closest to the target.
        """
        if target is not np.array:
            target = np.array(target)
        positions = [np.round(i.ee, 5) for i in neigh_nodes]
        hypotenuses = [target - n for n in positions]
        distances = [np.sqrt(np.sum(np.square(n))) for n in hypotenuses]
        return distances.index(min(distances))
            


    def generate_path(self, start_node: Node, target=[0, 0, 0], target_radius=0.01, max_iter=1500, filename="samplepath.csv"):
        
        def check_position_tolerance(heuristic, epsilon):
            return (heuristic < epsilon)
        
        
        path_q = queue.Queue()
        i = 0
        curr_node = start_node
        path_q.put((curr_node.l, curr_node.l_tendon))
        
        # Delta length to change each iteration
        del_length = 0.0009
       
        while (i <= max_iter):
            curr_node.run_forward_model(self, True, "KINEMATIC_CPP")
            
            if check_position_tolerance(np.linalg.norm(curr_node.ee-target), target_radius):
                print(f"Goal found at {target}")
                break
            
            
            neigh_traverse = [(del_length, 0), (-del_length, 0), (0, del_length), (0, -del_length), (del_length, del_length)]
            neighbour_list = [Node(curr_node.robotobj, curr_node.l +i[0], curr_node.l_tendon + i[1] ) for i in neigh_traverse]
            
            for neigh in neighbour_list:
                neigh.set_init_guess(curr_node.var[0,::3])
                exitflag = neigh.run_forward_model(self, True, "KINEMATIC_CPP")
                if exitflag:
                    pass
                else:
                    print(neigh.l, neigh.l_tendon, "Model computation failed")
           # import pdb; pdb.set_trace()
            next_node = neighbour_list[self.next_path_heuristic(neighbour_list, target)]
            path_q.put((next_node.l, next_node.l_tendon))
            curr_node = next_node
            print(f"Iteration {i}, position: {curr_node.ee}", (next_node.l, next_node.l_tendon))
            i += 1

    
        with open(filename, "w+") as f:
            while not path_q.empty():
                values = path_q.get()
                data = f"{round(values[0], 3)}, {round(values[1], 3)}\n"
                f.write(data)
        print(f'Written path too: {filename}')
        