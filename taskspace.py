import numpy as np
import matplotlib.pyplot as plt
from obstacle_definition import SuperEllipse
from utils import mod_cr

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

class TaskSpaceSuperEllipse(TaskSpace):
    def __init__(self):
        super().__init__()
        self.obstacle = []
        self.ob_center = []
        self.ob_ab = []
        self.abM = []
        self.ob_centerM = [] 
        

    def set_obstacles(self, ellipse : SuperEllipse, num_copies, delta):
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
            self.obstacle += [SuperEllipse(ellipse.ab, add_list(ellipse.p,[0,0,i*delta]))]
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
