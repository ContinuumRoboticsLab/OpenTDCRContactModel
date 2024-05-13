import numpy as np
import matplotlib.pyplot as plt
from utils import mod_cr
from utils_model import model_definition as md
    
class Node:
    def __init__(self, robot : mod_cr.Robot, l, actuation, model_type):
        """
        A node is used to represent the state of the TDCR at a given configuration.  

        Attributes:
        robot (object): An instance of the class robot, contains robot dimensions
        l (float): (Actuation space) Length of robot 
        actuation (float):  (Actuation space) Tendon length of robot at [r,0,0]
        model_type (str): The type of kinematic model used for the robot - either KINEMATIC_CPP or KINEMATIC_MATLAB
        var (numpy array): An array of size 3n, n = number of disks, stores the curvature of each subsegment
        """
        if l <= 0: raise Exception("Actuation space length must be greater than 0")
        if actuation <=0: raise Exception("Actuation space Tendon length must be greater than 0")
        


        self.__dict__.update(robot.__dict__)
        self.var = np.zeros((1,3*self.nd))
        self.l = round(l, 4)
        self.l_tendon = round(mod_cr.matlab_convertor(actuation),4)
        self.model_dict = {"KINEMATIC_MATLAB":md.KinematicMatlabModel(),
                        "KINEMATIC_CPP":md.KinematicCppModel()}
        self.exitflag = False
        
        if model_type not in self.model_dict.keys(): raise Exception("Model type invalid")



    def set_init_guess(self, x_init):
        # Sets the initial guess for th e robot's configuration
        self.x_init = mod_cr.matlab_convertor(x_init)
        self.x_init_python = x_init
        
    def set_config_bound(self, taskspace, x):
        """
        This method is used to limit the number of obstacles considered by the model to ones close to the initial guess
        
        Parameters:
        taskspace (object): An instance of the task space where the node is located.
        x (numpy array): The initial configuration of the node.

        Raises:
        Exception: If the initial configuration 'x' is not set for the Node.

        Returns:
        w_node (taskspace): An instance of the task space with the obstacles close to the shape resulting from the initial guess.
        """
        if type(x) == None:
            raise Exception("Initial configuration not set for Node")
        var_init = np.zeros((1,3*self.nd))
        var_init[0,::3] = x[:self.nd]
        coordinates, _ = mod_cr.positionCalc(self.l/self.nd, self.nd, var_init, self.tendon)
        # return coordinates
        w_node = type(taskspace)()

        for ob in taskspace.obstacle:
            obstacle_toggle = True
            p = ob.xy_gen()[:3,:]
            for i in range(self.nd):
                if np.sum(np.linalg.norm(np.array(p) - coordinates[:, i].reshape(3,1), axis = 0) < 2.5 * self.radius) > 0 :
                    if obstacle_toggle:
                        w_node.set_obstacles(ob, 1, 0.0)
                        obstacle_toggle = False

        if len(w_node.obstacle) == 0:
            w_node.set_obstacles(taskspace.obstacle[0], 1, 0.0)
        return w_node



    def set_parent(self, x ):
        self.parent = x

    def plot_configuration(self, taskspace, color = 'slateblue', target = False):
        taskspace.plot_workspace(target)
        mod_cr.plotTDCR(self.l/self.nd, self.tendon, self.nd, self.var, self.radius, color)
        mod_cr.plotSettings()
    
    def run_forward_model(self, taskspace, bool_reduced, model_type):
        #bool_reduced is a boolean that determines if the obstacles considered by the model are limited to ones that are close to the robot body
        if bool_reduced:
            w_reduced = self.set_config_bound(taskspace, self.x_init_python)
        else:
            w_reduced = taskspace
        

        xSol, _, exitflag = self.model_dict[model_type].run_model(self, w_reduced)
        self.exitflag = exitflag
        # extiflag returns True for a successful convergence of the solver
        if exitflag <=0 :
            return False
        self.model_dict[model_type].set_var(self, np.array(np.array(xSol)[0]))
        if (abs(self.l_tendon_calculated[0] - self.l_tendon) > 1e-4) :
            print(self.l_tendon_calculated, self.l_tendon, exitflag)
            raise RuntimeError("Solver doesnt satisfy tolerance of calculated tendon length being equal to desired actuated length")
        return True
    
