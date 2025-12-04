import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation as R
from matplotlib.animation import FuncAnimation
import pandas as pd
import sys
import os

# --- CONFIGURATION ---
ANIMATION_INTERVAL_MS = 33
DATA_STEP = 1
SMOOTHING_WINDOW = 5 

class BoxVisualizer:
    def __init__(self, data_file, interval=33, step=1):
        self.data_file = data_file
        self.interval = interval
        self.step = step
        self.paused = False
        
        self.last_idx = -1
        self.last_rot = None 
        self.current_orientation = R.identity()
        
        self.load_and_process_data()
        self.setup_geometry()
        self.setup_plot()

    def load_and_process_data(self):
        if not os.path.exists(self.data_file):
            print(f"File '{self.data_file}' not found. Generating dummy data.")
            t = np.linspace(0, 10, 500)
            dummy_euler = np.stack([0.5 * np.sin(t), 0.5 * np.cos(t), np.zeros_like(t)], axis=1)
            quats = R.from_euler('xyz', dummy_euler).as_quat()
        else:
            try:
                data = pd.read_csv(self.data_file)
                data.columns = [c.strip().lower() for c in data.columns]
                
                req_filtered = ['f_qx', 'f_qy', 'f_qz', 'f_qw']
                req_std = ['qx', 'qy', 'qz', 'qw']
                
                if set(req_filtered).issubset(data.columns):
                    print(f"Using filtered columns: {req_filtered}")
                    quats = data[req_filtered].values
                elif set(req_std).issubset(data.columns):
                    print(f"Using standard columns: {req_std}")
                    quats = data[req_std].values
                else:
                    print("No headers found. Assuming first 4 columns are [w, x, y, z]...")
                    raw = data.iloc[:, 0:4].values
                    quats = raw[:, [1, 2, 3, 0]]
            except Exception as e:
                print(f"Error reading CSV: {e}")
                sys.exit(1)

        # Smoothing
        if SMOOTHING_WINDOW > 1:
            df_quats = pd.DataFrame(quats)
            quats = df_quats.rolling(window=SMOOTHING_WINDOW, min_periods=1, center=True).mean().values
            norms = np.linalg.norm(quats, axis=1, keepdims=True)
            quats = quats / norms

        # Enforce Roll->Pitch (Yaw=0)
        temp_rot = R.from_quat(quats)
        eulers = temp_rot.as_euler('xyz', degrees=False)
        eulers[:, 2] = 0 
        
        self.rotations = R.from_euler('xyz', eulers)
        self.total_rows = len(self.rotations)

    def setup_geometry(self):
        r = 0.5 # Cube half-width
        
        # 1. Base Cube
        self.base_verts = np.array([
            [[-r, -r, -r], [ r, -r, -r], [ r,  r, -r], [-r,  r, -r]], 
            [[-r, -r,  r], [ r, -r,  r], [ r,  r,  r], [-r,  r,  r]], 
            [[-r, -r, -r], [-r,  r, -r], [-r,  r,  r], [-r, -r,  r]], 
            [[ r, -r, -r], [ r,  r, -r], [ r,  r,  r], [ r, -r,  r]], 
            [[-r, -r, -r], [-r, -r,  r], [ r, -r,  r], [ r, -r, -r]], 
            [[-r,  r, -r], [-r,  r,  r], [ r,  r,  r], [ r,  r, -r]], 
        ])

        # 2. Axis Geometry (OFFSET to appear outside)
        # Instead of 0 to 1, we go from 0.5 (surface) to 1.2 (outside)
        start_offset = 0.5 
        end_length = 1.2
        
        self.axis_starts = np.array([
            [start_offset, 0, 0], # X start (at surface)
            [0, start_offset, 0], # Y start (at surface)
            [0, 0, start_offset]  # Z start (at surface)
        ])
        
        self.axis_ends = np.array([
            [end_length, 0, 0],   # X end
            [0, end_length, 0],   # Y end
            [0, 0, end_length]    # Z end
        ])

    def setup_plot(self):
        self.fig = plt.figure(figsize=(10, 7))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_box_aspect((1, 1, 1)) # Fix aspect ratio
        
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.view_init(elev=30., azim=45.)
        
        # Opaque Cube
        self.cube_collection = Poly3DCollection(
            self.base_verts, 
            facecolors=['r','g','b','y','c','m'], 
            edgecolors='k', 
            alpha=1.0, 
            linewidths=1.5,
            zsort='average'
        )
        self.ax.add_collection3d(self.cube_collection)

        # Lines (Thicker for visibility)
        self.line_x, = self.ax.plot([], [], [], 'r', lw=4, label='X (Roll)')
        self.line_y, = self.ax.plot([], [], [], 'g', lw=4, label='Y (Pitch)')
        self.line_z, = self.ax.plot([], [], [], 'b', lw=4, label='Z (Yaw)')
        
        self.ax.legend(loc='upper right')
        
        self.pause_text = self.ax.text2D(0.05, 0.95, "PAUSED", transform=self.ax.transAxes, 
                                         color='red', fontsize=14, fontweight='bold')
        self.pause_text.set_visible(False)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)

    def on_key_press(self, event):
        if event.key == ' ':
            self.toggle_pause()

    def toggle_pause(self):
        if self.paused:
            self.anim.event_source.start()
            self.pause_text.set_visible(False)
            self.paused = False
        else:
            self.anim.event_source.stop()
            self.pause_text.set_visible(True)
            self.paused = True
        self.fig.canvas.draw_idle()

    def update(self, frame_idx):
        target_rot = self.rotations[frame_idx]
        
        if frame_idx == 0 or frame_idx < self.last_idx:
            self.current_orientation = target_rot
            self.last_rot = target_rot
        else:
            delta_rot = target_rot * self.last_rot.inv()
            self.current_orientation = delta_rot * self.current_orientation
            self.last_rot = target_rot

        self.last_idx = frame_idx

        # 1. Update Cube (Rotate Base Vertices)
        current_verts = [self.current_orientation.apply(face) for face in self.base_verts]
        self.cube_collection.set_verts(current_verts)

        # 2. Update Axes (Rotate Start AND End points)
        # This moves the whole vector stick along with the cube face
        rot_starts = self.current_orientation.apply(self.axis_starts)
        rot_ends = self.current_orientation.apply(self.axis_ends)

        # Update Lines: Draw from Rotated Start -> Rotated End
        self.line_x.set_data([rot_starts[0, 0], rot_ends[0, 0]], [rot_starts[0, 1], rot_ends[0, 1]])
        self.line_x.set_3d_properties([rot_starts[0, 2], rot_ends[0, 2]])
        
        self.line_y.set_data([rot_starts[1, 0], rot_ends[1, 0]], [rot_starts[1, 1], rot_ends[1, 1]])
        self.line_y.set_3d_properties([rot_starts[1, 2], rot_ends[1, 2]])
        
        self.line_z.set_data([rot_starts[2, 0], rot_ends[2, 0]], [rot_starts[2, 1], rot_ends[2, 1]])
        self.line_z.set_3d_properties([rot_starts[2, 2], rot_ends[2, 2]])

        return self.cube_collection, self.line_x, self.line_y, self.line_z

    def run(self):
        frame_indices = np.arange(0, self.total_rows, self.step)
        print(f"Starting Animation. Frames: {len(frame_indices)}")
        
        self.anim = FuncAnimation(
            self.fig, 
            self.update, 
            frames=frame_indices, 
            interval=self.interval, 
            blit=False
        )
        plt.show()

if __name__ == "__main__":
    def get_keyword():
        if len(sys.argv) > 1: return sys.argv[1]
        return 'test'

    def get_filenames(keyword):
        return f'./data/{keyword}/measured.txt', f'./data/{keyword}/filtered.txt'

    keyword = get_keyword()
    measured_file, filtered_file = get_filenames(keyword)
    
    viz = BoxVisualizer(filtered_file, interval=ANIMATION_INTERVAL_MS, step=DATA_STEP)
    viz.run()