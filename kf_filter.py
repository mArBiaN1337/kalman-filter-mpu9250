import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import sys

def strip_argv():
    """Strip non-essential arguments from sys.argv for easier testing."""
    # take the keyword of the string "python kf_filter.py <keyword>"
    args = sys.argv
    if len(args) > 1:
        return args[1]
    else:
        return 'test'

def dataset():
    # generate the strings for the datasets according to the keyword
    keyword = strip_argv()
    
    folder_path = f'./data/{keyword}'
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
        open(f'./data/{keyword}/measured.txt', 'w').close()
        open(f'./data/{keyword}/filtered.txt', 'w').close()
        open(f'./data/{keyword}/quaternions.txt', 'w').close()
    else:
        pass   

    return f'./data/{keyword}/measured.txt', f'./data/{keyword}/filtered.txt', f'./data/{keyword}/quaternions.txt'

FILENAMES = dataset()  
PLOT_OPTION = True

ANGLE_UNITS = 'deg','rad'
ANGLE_CHOICE = ANGLE_UNITS[0]

class KalmanFilter:
    def __init__(self):
        plt.close('all')

        self.imu_data = None
        self.predicted_data = None
        self.total_samples = 0

        self.accel = np.array([[0., 0., 0.]])  # ax, ay, az
        self.gyro = np.array([[0., 0., 0.]])  # gx, gy, gz

        self.euler_attitude = np.array([[0., 0., 0.]])  # roll, pitch, yaw
        self.measured_quaternion = np.array([[1., 0., 0., 0.]])  # qw, qx, qy, qz

        self.measured_attitude = np.array([[0., 0., 0.]])
        self.filtered_attitude = np.array([[0., 0., 0.]])

        self.A = np.eye(4)
        self.C = np.eye(4)

        self.Q = 0.  # process noise covariance
        self.R = 0.  # measurement noise covariance

        self.process_noise_covar = np.eye(4) * self.Q
        self.measurement_noise_covar = np.eye(4) * self.R

        self.initial_uncertainty = np.eye(4)

        self.sampling_frequency = 100.0  # Hz
        self.dt = 1.0 / self.sampling_frequency

    def filter(self):

        self.process_noise_covar = np.eye(4) * self.Q
        self.measurement_noise_covar = np.eye(4) * self.R
        
        x_old = np.array([[1., 0., 0., 0.]]).T  # initial state
        P_old = self.initial_uncertainty  # initial uncertainty

        with open(FILENAMES[1], 'w') as f:
            f.write("m_roll,m_pitch,m_yaw,f_roll,f_pitch,f_yaw\n")

        with open(FILENAMES[2], 'w') as f:
            f.write("m_qw,m_qx,m_qy,m_qz,f_qw,f_qx,f_qy,f_qz\n")

        for i in range(self.total_samples):

            ax = self.imu_data['ax'][0][i]
            ay = self.imu_data['ay'][0][i]
            az = self.imu_data['az'][0][i]

            gx = self.imu_data['gx'][0][i]
            gy = self.imu_data['gy'][0][i]
            gz = self.imu_data['gz'][0][i]

            self.accel = np.array([[ax, ay, az]])
            self.gyro = np.array([[gx, gy, gz]])
            
            self.fix_orientation()

            self.euler_attitude = self.roll_pitch(*self.accel[0])
            self.measured_quaternion = self.euler_to_quat(*self.euler_attitude[0])

            norm = np.linalg.norm(self.measured_quaternion)
            if norm > 1e-6:
                self.measured_quaternion = self.measured_quaternion / norm
            else:
                pass                
            
            gx, gy, gz = self.gyro[0]
            Un = np.array([[0, -gx, -gy, -gz],
                          [gx, 0, gz, -gy],
                          [gy, -gz, 0, gx],
                          [gz, gy, -gx, 0]])
            
            self.A = np.eye(4) + (0.5 * self.dt * Un)
            
            x_pred = self.A @ x_old
            P_pred = self.A @ P_old @ self.A.T + self.process_noise_covar

            K_gain = P_pred @ self.C.T @ np.linalg.inv(self.C @ P_pred @ self.C.T + self.measurement_noise_covar)

            if np.dot(x_pred.T, self.measured_quaternion.T) < 0:
                self.measured_quaternion = -self.measured_quaternion

            x_new = x_pred + K_gain @ (self.measured_quaternion.T - self.C @ x_pred)
            P_new = (np.eye(4) - K_gain @ self.C) @ P_pred

            norm = np.linalg.norm(x_new)
            if norm > 1e-6:
                x_new = x_new / norm
            else:
                pass

            x_old = x_new
            P_old = P_new

            self.measured_attitude = self.quat_to_euler(*self.measured_quaternion[0])
            self.filtered_attitude = self.quat_to_euler(*x_new.T[0])

            self.append_angles(self.measured_attitude, self.filtered_attitude)
            self.append_quaternion(self.measured_quaternion, x_new.T)
    
    def fix_orientation(self):
        rotation_matrix = np.array([[-1, 0, 0],
                                        [0, 1, 0],
                                        [0, 0, -1]])
            
        self.accel = (rotation_matrix @ self.accel.T).T
        self.gyro = (rotation_matrix @ self.gyro.T).T

    def append_angles(self, measured, filtered):
        try:
            with open(FILENAMES[1], 'a') as f:
                f.write(f"{measured[0,0]:<.3f},{measured[0,1]:<.3f},{measured[0,2]:<.3f},{filtered[0,0]:<.3f},{filtered[0,1]:<.3f},{filtered[0,2]:<.3f}\n")

        except Exception as e:
            pass
    
    def append_quaternion(self, measured, filtered):
        try:
            with open(FILENAMES[2], 'a') as f:
                f.write(f"{measured[0,0]:<.6f},{measured[0,1]:<.6f},{measured[0,2]:<.6f},{measured[0,3]:<.6f},{filtered[0,0]:<.6f},{filtered[0,1]:<.6f},{filtered[0,2]:<.6f},{filtered[0,3]:<.6f}\n")

        except Exception as e:
            pass

    @staticmethod
    def gyro_to_euler(gx, gy, gz, roll, pitch):
        # represent in matrix form
        G = np.array([[gx, gy, gz]])

        R = np.array([[1, np.sin(roll) * np.tan(pitch), np.cos(roll) * np.tan(pitch)],
                      [0, np.cos(roll), -np.sin(roll)],
                      [0, np.sin(roll) / np.cos(pitch), np.cos(roll) / np.cos(pitch)]])
        
        corr_gyro = R @ G.T

        return corr_gyro.T

    @staticmethod
    def roll_pitch(ax, ay, az):
        roll = np.arctan2(ay, az)
        pitch = np.arctan2(-ax, np.sqrt(ay * ay + az * az))

        return np.array([[roll, pitch, 0.]])

    @staticmethod
    def euler_to_quat(roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return np.array([[qw, qx, qy, qz]])
    
    @staticmethod
    def quat_to_euler(qw, qx, qy, qz):
        # roll (x-axis rotation)
        t0 = 2.0 * (qw * qx + qy * qz)
        t1 = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = np.arctan2(t0, t1)

        t2 = 2.0 * (qx * qz - qw * qy)
        t3 = 1.0 - 2.0 * (qx * qx + qy * qy)
        pitch = np.arctan2(-t2, t3)

        # yaw (z-axis rotation)
        t4 = 2.0 * (qw * qz + qx * qy)
        t5 = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = np.arctan2(t5, t4)

        # make angles between 0 and 2pi
        roll = np.mod(roll, 2 * np.pi)
        pitch = np.mod(pitch, 2 * np.pi)
        yaw = np.mod(yaw, 2 * np.pi)
        
        return np.array([[roll, pitch, yaw]])
    
    def read_sensor(self):
        try:
            data = np.loadtxt(FILENAMES[0], delimiter=',', skiprows=1)
            self.imu_data = {k: data[:, i] for i, k in enumerate(['ax', 'ay', 'az', 'gx', 'gy', 'gz'])}
            self.imu_data = {k: np.array([v]) for k, v in self.imu_data.items()}

            self.total_samples = np.size(self.imu_data['ax'][0])

        except Exception as e:
            self.imu_data = None

        self.accel_cov = np.cov([self.imu_data['ax'].flatten(), self.imu_data['ay'].flatten(), self.imu_data['az'].flatten()])
        self.gyro_cov = np.cov([self.imu_data['gx'].flatten(), self.imu_data['gy'].flatten(), self.imu_data['gz'].flatten()])

        #self.Q = (np.trace(self.gyro_cov) * 1.0) * (self.dt ** 1)
        #self.R = (np.trace(self.accel_cov) * 1.0) * (1.0/(self.dt ** 1))
        self.Q = 0.003 ** 2
        self.R = 0.05 ** 2

    def read_results(self):
        try:
            data = np.loadtxt(FILENAMES[1], delimiter=',', skiprows=1)
            self.predicted_data = {k: data[:, i] for i, k in enumerate(['m_roll', 'm_pitch', 'm_yaw', 'f_roll', 'f_pitch', 'f_yaw'])}
            self.predicted_data = {k: np.array([v]) for k, v in self.predicted_data.items()}

        except Exception as e:
            self.predicted_data = None

    def calibrate(self, CALIBRATION_FILE='./data/calibration/measured.txt'):
        
        try:
            data = np.loadtxt(CALIBRATION_FILE, delimiter=',', skiprows=1)
            calibration_data = {k: data[:, i] for i, k in enumerate(['ax', 'ay', 'az', 'gx', 'gy', 'gz'])}
            calibration_data = {k: np.array([v]) for k, v in calibration_data.items()}

        except:
            pass            

        params = {'bias': {}, 'scale': 0.0}
        for axis in ['gx', 'gy', 'gz', 'ax', 'ay', 'az']:
            if axis in ['az']:
                params['bias'][axis] = np.abs(np.abs(np.mean(calibration_data[axis][0])) - 9.81)
            else:
                params['bias'][axis] = np.abs(np.mean(calibration_data[axis][0]))

        for axis in ['gx', 'gy', 'gz', 'ax', 'ay']:
            calibration_data[axis][0] = calibration_data[axis][0] - params['bias'][axis]
            
        params['scale'] = (np.max(calibration_data['az'][0]) - np.min(calibration_data['az'][0]))/2

        for param in ['bias', 'scale']:
            for axis in ['gx', 'gy', 'gz', 'ax', 'ay', 'az']:
                if param == 'bias':
                    self.imu_data[axis][0] = self.imu_data[axis][0] - params['bias'][axis]

                elif param == 'scale' and axis in ['ax', 'ay', 'az']:
                    if params['scale'] > 1:
                        self.imu_data[axis][0] = self.imu_data[axis][0] * params['scale']
                    elif params['scale'] < 1:
                        self.imu_data[axis][0] = self.imu_data[axis][0] / params['scale']
                    else: 
                        pass

    def DLPF(self, rel_cutoff_frequency=0.05):
        nyquist_freq = 0.5 * self.sampling_frequency
        cutoff_freq = rel_cutoff_frequency * self.sampling_frequency

        b, a = butter(N=2, Wn=cutoff_freq / nyquist_freq, btype='low')

        for axis in ['ax', 'ay', 'az', 'gx', 'gy', 'gz']:
            self.imu_data[axis][0] = filtfilt(b, a, self.imu_data[axis][0])

    def plot_results(self, plot_option=PLOT_OPTION): 
        sample_size = self.total_samples - 1

        if plot_option:
            sample_size = sample_size + 1

            m_roll_unwrapped = np.unwrap(self.predicted_data['m_roll'][0,:sample_size])
            f_roll_unwrapped = np.unwrap(self.predicted_data['f_roll'][0,:sample_size])
            
            m_pitch_unwrapped = np.unwrap(self.predicted_data['m_pitch'][0,:sample_size])
            f_pitch_unwrapped = np.unwrap(self.predicted_data['f_pitch'][0,:sample_size])

            if ANGLE_CHOICE == 'deg':
                ylabel = ' (deg)'
                m_roll_plot = m_roll_unwrapped * 180/np.pi
                f_roll_plot = f_roll_unwrapped * 180/np.pi
                m_pitch_plot = m_pitch_unwrapped * 180/np.pi
                f_pitch_plot = f_pitch_unwrapped * 180/np.pi

            elif ANGLE_CHOICE == 'rad':
                ylabel = ' (rad)'
                m_roll_plot = m_roll_unwrapped
                f_roll_plot = f_roll_unwrapped
                m_pitch_plot = m_pitch_unwrapped
                f_pitch_plot = f_pitch_unwrapped

            #make it so I don't adjust it every time
            plt.figure(figsize=(12, 8), tight_layout=True, facecolor='w', edgecolor='k', num='Kalman Filter Results')
            manager = plt.get_current_fig_manager()
            manager.window.geometry("+0+0")
            plt.subplots_adjust(hspace=0.5)

            plt.subplot(2, 1, 1)
            plt.plot(np.arange(sample_size), m_roll_plot, label='measured_roll', color='r', linestyle='dashed')

            plt.plot(np.arange(sample_size), f_roll_plot, label='filtered_roll', color='r')

            plt.legend(loc='upper right')
            plt.ylabel('roll' + ylabel)
            #make it show from -180 to 180 degrees and ticks every 45 degrees
            #plt.yticks(np.arange(-180, 190, 45))
            #plt.ylim([-200, 210])
            plt.grid(which='both', linestyle='--', linewidth=0.5)
            plt.minorticks_on()

            plt.subplot(2, 1, 2)
            plt.plot(np.arange(sample_size), m_pitch_plot, label='measured_pitch', color='g', linestyle='dashed')

            plt.plot(np.arange(sample_size), f_pitch_plot, label='filtered_pitch', color='g')

            plt.legend(loc='upper right')
            plt.ylabel('pitch' + ylabel)
            #plt.yticks(np.arange(-180, 190, 45))
            #plt.ylim([-200, 210])
            plt.grid(which='both', linestyle='--', linewidth=0.5)
            plt.minorticks_on()

            plt.show()
   
    
if __name__ == '__main__':
    np.set_printoptions(precision=3, suppress=True)

    kf = KalmanFilter()
    kf.read_sensor()
    kf.calibrate()
    kf.DLPF()

    kf.filter()
    kf.read_results()
    kf.plot_results()