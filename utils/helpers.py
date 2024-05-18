
import numpy as np
from pylab import text
from celluloid import Camera
import dill
import matplotlib.pyplot as plt
import cv2, os

def save_object(ob, filename):
    dill.dump(ob, file = open(filename+".pickle", "wb"))

def calculate_signed_distance(p1, p2, point):
        return (point[0]-p1[0])*(p2[1]-p1[1]) -  (point[1]-p1[1])*(p2[0]-p1[0])


def load_object(filename):
    return dill.load(open(filename+".pickle", "rb"))


def visualizing(traced_path, workspace, filename, show_video=False):
    """
    Visualizes the final traced path
    """
    fig = plt.figure(dpi=150)
    size = tuple(fig.get_size_inches()*fig.dpi)
    camera = Camera(fig)
    skip_step = 2 if len(traced_path) > 100 else 1

    for i, curr_node in enumerate(traced_path[::-1]):#[::-1]:
        if (i % skip_step == 0 ):
            if i ==0:
                curr_node.plot_configuration(workspace)
            else:
                curr_node.plot_configuration(workspace, 'slateblue', False)
            # text(0.005, 0.01, "Node : "+str(i), fontsize=12)
            camera.snap()


    for i in range(0,50):
        traced_path[0].plot_configuration(workspace)
        # text(0.005, 0.01, "Node : "+str(len(traced_path)), fontsize=12)
        camera.snap()
    if show_video:
        plt.show()
    animation = camera.animate()
    animation.save(filename + '.mp4', writer='ffmpeg', fps = 15)
    # #animation.save(filename + '.gif', writer='Pillow', fps=25)
    
    
    cap = cv2.VideoCapture(filename + '.mp4')
   
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter("media/path.mp4", fourcc, 20,(int(size[0]),int(size[1])))
   
    clogo = cv2.imread("media/crlLogo.png")
    ulogo = cv2.imread("media/uoftLogo.png")   
    print("Writing video")
    while True:
        ret, frame = cap.read()
        if ret:
            frame[0: clogo.shape[0], 0: clogo.shape[1]] = clogo
            frame[0: ulogo.shape[0], 860: 860+ulogo.shape[1]] = ulogo
            out.write(frame)
            # cv2.imshow('frame', frame)
        else:
            break

    #os.remove("media/sample.mp4")



def saveFigure():
    plt.savefig('media/initial_config.png')
    im = cv2.imread('media/initial_config.png')
    
    clogo = cv2.imread("media/crlLogo.png")
    clogo = cv2.resize(clogo, (0, 0), fx = 0.8, fy = 0.8)
    ulogo = cv2.imread("media/uoftLogo.png") 
    ulogo = cv2.resize(ulogo, (0, 0), fx = 0.8, fy = 0.8)
    im[0: clogo.shape[0], 0: clogo.shape[1]] = clogo
    im[0: ulogo.shape[0], im.shape[1]-ulogo.shape[1]: im.shape[1],:] = ulogo
    cv2.imwrite('media/initial_config.png', im)