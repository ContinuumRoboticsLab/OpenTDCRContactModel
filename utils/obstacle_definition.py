from abc import abstractmethod
import numpy as np
import matplotlib.pyplot as plt
from math import pi

class Obstacle:
    """
    Base class definition for obstacle
    """
    @abstractmethod
    def plot(self):
        raise NotImplementedError

    @abstractmethod
    def distance_to_obstacle(self, point):
        raise NotImplementedError

    @abstractmethod
    def xy_gen(self):
        raise NotImplementedError
    
        
    def plot(self):
        p = self.xy_gen()
        ax = plt.gca()
        plt.plot(p[0,:], p[2,:], color = (3/256,76/256,178/256),alpha = 0.2,linewidth=1)
        ax.fill(p[0,:],p[2,:], color = (3/256,76/256,178/256), alpha = 0.2, edgecolor='none')
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
    

class Circle(Obstacle):
    def __init__(self, radius, obcenter):
        #import pdb; pdb.set_trace()
        if type(radius) == tuple:
            self.ab = radius
        else:
            self.ab = (radius, radius, 2)
        self.p = obcenter

    def xy_gen(self):
        [a,b,s] = self.ab
        theta = np.linspace(0, 2*pi, 100)
        r = np.divide((a*b),((b*np.cos(theta))**s+(a*np.sin(theta))**s)**(1/s))          
        x = self.p[0]+r*np.cos(theta) 
        z = self.p[2]+r*np.sin(theta)
        return np.array([x,np.zeros(x.shape),z,np.ones(np.shape(x))])
       
    def distance_to_obstacle(self, point):
        """
        Takes a 4xN or 3xN matrix and returns distance from given obstacle"""
        [a,b,s] = self.ab
        cCenter = np.array([self.p[0:3]]).T
        v1 = ((point-cCenter))
        v2 = (np.array([[a],[1],[b]]))
        v3 = np.power(np.divide(v1,v2),s)
        return np.sum(v3,axis=0)-1

    def get_normal(self, point):
        [a,b,n] = self.ab
        normal = np.array([n*((point[0] - self.p[0])/a)**(n-1),0, n*((point[2] - self.p[2])/b)**(n-1)])
        return normal/np.linalg.norm(normal)
