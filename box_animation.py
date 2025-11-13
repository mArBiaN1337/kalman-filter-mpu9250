# build a animation of the box moving according to the quaternion data from the filtered data file
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation as R
import pandas as pd
import sys
 

def strip_argv():
    """Strip non-essential arguments from sys.argv for easier testing."""
    essential_args = {'roll': 0, 'pitch': 1, 'test': 2}
    keyword = sys.argv[1] if len(sys.argv) > 1 else 'roll'
    return essential_args.get(keyword, 0)

TEST = './data/test/measured.txt', './data/test/filtered.txt'
ROLL = './data/roll_data/measured.txt', './data/roll_data/filtered.txt'
PITCH = './data/pitch_data/measured.txt', './data/pitch_data/filtered.txt'
DATA_SETS = [ROLL, PITCH, TEST]
DATA_SET_INDEX = strip_argv()
DATA_SET_INDEX = int(DATA_SET_INDEX)
FILENAMES = DATA_SETS[DATA_SET_INDEX] 

def plot_box(ax, position, orientation):
    # Define the vertices of a unit cube centered at the origin
    # make the cube's face all connected

    r = R.from_quat(orientation)
    cube_definition = [
        np.array([[-0.5, -0.5, -0.5],
                    [0.5, -0.5, -0.5],
                    [0.5, 0.5, -0.5],
                    [-0.5, 0.5, -0.5]]),
        np.array([[-0.5, -0.5, 0.5],
                    [0.5, -0.5, 0.5],
                    [0.5, 0.5, 0.5],
                    [-0.5, 0.5, 0.5]]),
        np.array([[-0.5, -0.5, -0.5],
                    [-0.5, 0.5, -0.5],
                    [-0.5, 0.5, 0.5],
                    [-0.5, -0.5, 0.5]]),
        np.array([[0.5, -0.5, -0.5],
                    [0.5, 0.5, -0.5],
                    [0.5, 0.5, 0.5],
                    [0.5, -0.5, 0.5]]),
        np.array([[-0.5, -0.5, -0.5],
                    [-0.5, -0.5, 0.5],
                    [0.5, -0.5, 0.5],
                    [0.5, -0.5, -0.5]]),
        np.array([[-0.5, 0.5, -0.5],
                    [-0.5, 0.5, 0.5],
                    [0.5, 0.5, 0.5],
                    [0.5, 0.5, -0.5]]),
    ]
    # Rotate and translate the cube
    cube_definition = [r.apply(cube) + position for cube in cube_definition]
    # Plot the faces
    face_colors = ['r', 'g', 'b', 'y', 'c', 'm']
    collection = Poly3DCollection(cube_definition, facecolors=face_colors, linewidths=1, edgecolors='k', alpha=0.5)
    ax.add_collection3d(collection)

    axis_length = 0.7
    origin = position
    # Define the traditional axes rotated by 180 degrees around y axis
    initial_orientation = R.from_quat([0, 0, 1, 0])
    x_axis = initial_orientation.apply([axis_length, 0, 0]) + origin
    y_axis = initial_orientation.apply([0, axis_length, 0]) + origin
    z_axis = initial_orientation.apply([0, 0, axis_length]) + origin

    ax.quiver(*origin, *(x_axis - origin), color='r', length=axis_length)
    ax.quiver(*origin, *(y_axis - origin), color='g', length=axis_length)
    ax.quiver(*origin, *(z_axis - origin), color='b', length=axis_length)
    # indicate the axes with names drawn from the arrow
    ax.text(*(x_axis + 0.1), 'X', color='r')
    ax.text(*(y_axis + 0.1), 'Y', color='g')
    ax.text(*(z_axis + 0.1), 'Z', color='b')


def animate_box_motion(data_file):
    plt.close('all')

    # Load data
    data = pd.read_csv(data_file)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Set plot limits
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])

    # clear the xyz lines also
    ax.plot([], [], [], 'k-')

    ax.grid(False)
    
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])

    ax.view_init(elev=30., azim=45.)

    for index, row in data.iterrows():
        ax.cla()  # Clear the axes for the new frame

        ax.grid(False)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_zticks([])

        # Set plot limits again after clearing
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        # the file has only roll and pitch angles but no yaw, so the box will not rotate around z axis
        position = np.array([0, 0, 0])  # Fixed position at origin
        # the angles are in radians
        roll = row['filtered_roll']
        pitch = row['filtered_pitch']
        yaw = 0.0

        orientation = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        plot_box(ax, position, orientation)

        plt.pause(1/60)  

    plt.show(block=True)

if __name__ == "__main__":
    animate_box_motion(FILENAMES[1])


    