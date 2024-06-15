
import numpy as np
from pylab import text
from celluloid import Camera
import dill
import matplotlib.pyplot as plt
import os
from PIL import Image 
from moviepy.editor import VideoFileClip, ImageClip, CompositeVideoClip

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
    fig = plt.figure(dpi=300)
    camera = Camera(fig)
    skip_step = 2 if len(traced_path) > 100 else 1

    for i, curr_node in enumerate(traced_path):#[::-1]:
        if (i % skip_step == 0 ):
            if i ==0:
                curr_node.plot_configuration(workspace)
            else:
                curr_node.plot_configuration(workspace, 'slateblue', False)
            camera.snap()


    for i in range(0,50):
        traced_path[0].plot_configuration(workspace)
        camera.snap()
    if show_video:
        plt.show()
    animation = camera.animate()
    animation.save(filename + '.mp4', writer='ffmpeg', fps = 15)
    #animation.save(filename + '.gif', writer='Pillow', fps=25)
    
    
   
    
    video = VideoFileClip(filename + '.mp4')
    logo =ImageClip("media/logo.png").set_duration(video.duration)
    logo = logo.set_pos(("left", "top"))
    video_with_watermark = CompositeVideoClip([video, logo])
    video_with_watermark.write_videofile("media/pathing.mp4", codec="libx264")
    os.remove(filename + '.mp4')

def saveFigure(filename):

    plt.savefig(f'media/{filename}')
    logo = Image.open("media/logo.png")
    figure = Image.open(f"media/{filename}")
    figure.paste(logo, (5, 5), mask = logo)
    figure.save(f'media/{filename}')