
import numpy as np
from pylab import text
from celluloid import Camera
import dill
import matplotlib.pyplot as plt

def save_object(ob, filename):
    dill.dump(ob, file = open(filename+".pickle", "wb"))

def calculate_signed_distance(p1, p2, point):
        return (point[0]-p1[0])*(p2[1]-p1[1]) -  (point[1]-p1[1])*(p2[0]-p1[0])


def load_object(filename):
    return dill.load(open(filename+".pickle", "rb"))

def visualizing(traced_path, workspace, filename):
    """
    Visualizes the final traced path
    """
    fig = plt.figure(dpi=300)
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
    plt.show()
    animation = camera.animate()
    animation.save(filename + '.mp4', writer='ffmpeg', fps = 20)
    # animation.save(filename + '.gif', writer='Pillow', fps=25)
