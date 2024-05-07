import numpy as np
import importlib
from utils import mod_cr
from abc import abstractmethod
import matlab.engine
eng = matlab.engine.start_matlab()
eng.addpath('./utils_model')
cpp_solver_module = importlib.import_module("utils_model.cpp.cmake-build-release.pcca_solver")


class Model:
    @abstractmethod
    def set_var(self, node, x):
        """
        Virtual method that is overridden in subclasses to set the curvature parameters, 
        coordinates, and end effector values of the node.

        Parameters:
        node (Node): The node for which the parameters are to be set
        x (array): The new values for the solved curvature parameters
        """
        pass

    @abstractmethod
    def run_model():
        pass

class KinematicMatlabModel(Model):
    # This class is used to run the kinematic model of the robot using the matlab implementation
    def run_model(self, node, workspace):
        workspace.matlab_conversion()

        xSol,lamb,exitflag = eng.pccaSolver(node.x_init,float(node.l/node.nd), float(node.nd),\
                                                float(node.radius),float(node.l_tendon),mod_cr.matlab_convertor(node.tendon)\
                                                    ,workspace.ob_centerM,workspace.abM,nargout=3)
        
        node.full_model_status = True
        return xSol, lamb, exitflag

    def set_var(self, node, x):
        node.var[0,::3] = x
        node.T = mod_cr.transMatrix(node.var, node.l/node.nd, node.nd, 0)
        node.coordinates, node.l_tendon_calculated = mod_cr.positionCalc( node.l/node.nd, node.nd, node.var, node.tendon)
        node.ee = node.T[0:3,3]

class KinematicCppModel(Model):
    # This class is used to run the kinematic model of the robot using the cpp implementation
    def __init__(self):
        super().__init__()

    def run_model(self, node, workspace):
        workspace.cpp_conversion()

        xSol_tmp,exitflag = cpp_solver_module.pcca_solver().solve(node.nd, node.l, node.l_tendon, node.radius, node.x_init_python, node.tendon, workspace.ab, workspace.s, workspace.ob_centerCpp)
        lamb=None
        xSol = np.array(xSol_tmp).reshape(1,node.nd)

        node.full_model_status = True
        return xSol, lamb, exitflag

    def set_var(self, node, x):
        node.var[0,::3] = x
        node.T = mod_cr.transMatrix(node.var, node.l/node.nd, node.nd, 0)
        node.coordinates, node.l_tendon_calculated = mod_cr.positionCalc( node.l/node.nd, node.nd, node.var, node.tendon)
        node.ee = node.T[0:3,3]

     