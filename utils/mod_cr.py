import numpy as np
import matplotlib.pyplot as plt
import math
#import matlab.engine


class Robot:
    """
    Struct to store the robot parameters
    """
    def __init__(self, radius, nd, E = 60e9, ro = 1e-3):
        if radius < 6e-10: raise Exception(f"Radius must be greater than: 6e-10, your radius is :{radius}")
        if nd <= 0: raise Exception(f"Must be atleast 1 disk, your nd is: {nd}")
        if E >= 60e30: raise Exception(f"Youngs modulus must be smaller than: 60e30")
        if ro <= 1e-20: raise Exception(f"Outer radius must be greater than 1e-20")



        self.radius = radius #radius of disk
        self.tendon = np.array([[self.radius,0,0,1],[-self.radius,0,0,1]]).transpose() #4x2 array of tendon positions in the disk
        self.nd = nd # number of disks
        self.del_length = 0.001
        self.del_tendon_length = 0.001 
        self.E = E #Youngs modulus
        self.ro = ro #outer radius of backbone
        
def transMatrix(var,lInter,q,p):
    """
    Returns the transformation matrix from frame q to p 
    """
    T=np.identity(4)
    if q<p:
        raise Exception("Error in the required frame indices. Need q>=p")
    else:
        for ss_i in range(p+1,q+1):
            [beta,gamma,epS]=var[0,3*(ss_i-1):3*(ss_i-1)+3]
            k=np.sqrt(beta**2+gamma**2)
            if k==0:
                k=0.0000001 #Accounting for numerical errors/overflow

            phi=math.atan2(gamma,beta)
            if hasattr(lInter, "__len__"):
                theta=k*lInter[ss_i-1]
            else:
                theta=k*lInter
            p_i=np.array([[(1-np.cos(theta))*np.cos(phi)],\
                [(1-np.cos(theta))*np.sin(phi)],\
                [np.sin(theta)]])/k
            Rz=np.array([[np.cos(phi),-np.sin(phi),0],\
                         [np.sin(phi),np.cos(phi),0],\
                         [0,0,1]])
            Rz2=np.array([[np.cos(epS-phi),-np.sin(epS-phi),0],\
             [np.sin(epS-phi),np.cos(epS-phi),0],\
             [0,0,1]])
            Ry=np.array([[np.cos(theta),0,np.sin(theta)],\
                          [0,1,0],
                         [-np.sin(theta),0,np.cos(theta)]])
            T=T@np.vstack((np.hstack((Rz@Ry@Rz2, p_i)), [0, 0, 0 ,1]))
            
    return T     

    
def matlab_convertor(x):
    if (isinstance(x, float)):
        return float(x)
    #if isinstance(x, list):
    #    return matlab.double(x)
    #return matlab.double(x.tolist())
    return np.array(x, dtype=np.double)

def positionCalc(lInter,nd,var,pTendon):
    """
    Returns 
    pCoord :the coorsindates of each disk
    lTendon: length of tendon1 and tendon2 pulling it
    ptCoord : coordinates of tendon position
    """
    pCoord = np.zeros((3,nd))
    ptCoord1 = np.zeros((3,2*nd+1));ptCoord2=np.zeros((3,2*nd+1))
    ptCoord1[:,0] = pTendon[0:3,0]
    ptCoord2[:,0] = pTendon[0:3,1]
    lTendon = np.array([0,0])
    Ti = np.identity(4)
    for ss_i in range(nd):
        Ti = Ti@transMatrix(var,lInter,ss_i+1,ss_i)
        pt = Ti@pTendon
        pCoord[:,ss_i] = Ti[0:3,3]
        ptCoord1[:,ss_i+1] = pt[0:3,0]
        ptCoord2[:,ss_i+1] = pt[0:3,1]
        ptCoord2[:,nd+ss_i+1] = (ptCoord2[:,ss_i+1]+ptCoord2[:,ss_i])/2
        ptCoord1[:,nd+ss_i+1] = (ptCoord1[:,ss_i+1]+ptCoord1[:,ss_i])/2
        lTendon = lTendon + np.array([np.sqrt(np.sum((ptCoord1[:,ss_i+1]-ptCoord1[:,ss_i])**2)),
                         np.sqrt(np.sum((ptCoord2[:,ss_i+1]-ptCoord2[:,ss_i])**2))])
    pC = np.concatenate((pCoord,ptCoord1[:,1:],ptCoord2[:,1:]),axis=1)
    return pC,lTendon

def plotEE(T):
        plt.plot( T[0,3],T[2,3],marker='o',markerfacecolor='g',markersize=3, markeredgecolor='g')
        plt.arrow(T[0,3], T[2,3], 0.005 * T[0,2], 0.01* T[2,2], color = 'g', width = 0.001, alpha = 0.5)

def plotTDCR(lInter,pTendon,nd,var,rDisk, color_tendon):
    """
    Simply plots the backbone in a figure
    """

    for ss_i in range(nd):
        [beta,gamma,epS]=var[0,3*ss_i:3*ss_i+3]
        k=np.sqrt(beta**2+gamma**2)
        phi=np.arctan2(gamma,beta)
        r=1/k
        if hasattr(lInter, "__len__"):
            s=np.asmatrix(np.linspace(0, lInter[ss_i], 100))
#     print(s)
        else:
            s=np.asmatrix(np.linspace(0, lInter, 100))
        n=s.shape[1]
        p_i=np.ones((4,n))
        p_i[0,:]=r*(1-np.cos(s/r))*np.cos(phi)
        p_i[1,:]=r*(1-np.cos(s/r))*np.sin(phi)
        p_i[2,:]=r*np.sin(s/r)

        T=transMatrix(var,lInter,ss_i,0)
        pPlot=T@p_i
        p_t1=T@pTendon

        pLast=np.asmatrix(pPlot[:,-1]).transpose()
        # plt.plot( pLast[0,:],pLast[2,:],'ko',markersize=1)
        
        T1=transMatrix(var,lInter,ss_i+1,0)
        p_t2=T1@pTendon
        if color_tendon == 'silver':
            plt.plot( pPlot[0,:],pPlot[2,:],color=color_tendon, linewidth = 2.5, alpha = 1)
            plt.plot( [p_t1[0],p_t2[0]], [p_t1[2],p_t2[2]],color=color_tendon,markersize=0, linewidth =2, alpha =1)
        else:
            plt.plot( pPlot[0,:],pPlot[2,:],color=(0,0,0), linewidth = 3.5)
            plt.plot( [p_t1[0],p_t2[0]], [p_t1[2],p_t2[2]],color=color_tendon,markersize=0, linewidth =2.5)
        ax = plt.gca()
        
        ax.fill( [p_t1[0][0], p_t2[0][0], p_t2[0][1],  p_t1[0][1]], [p_t1[2][0], p_t2[2][0],p_t2[2][1], p_t1[2][1]],color='gainsboro',  alpha = 0.3)
        # pt_prev_1, pt_prev_2 = p_t1, p_t2

    plt.plot([p_t2[0][0], p_t2[0][1]], [p_t2[2][0], p_t2[2][1]], color = 'k', linewidth = 1)
    plt.arrow(T1[0,3], T1[2,3], 0.01 * T1[0,2], 0.01* T1[2,2], color = 'g', width = 0.001, alpha = 0.5)
    plt.arrow(T1[0,3], T1[2,3], 0.01 * T1[0,0], 0.01* T1[2,0], color = 'r', width = 0.0005, alpha = 0.5)

    
def plotSettings():
    plt.plot([-0.02,0.02],[0,0],'k', linewidth = 2)
