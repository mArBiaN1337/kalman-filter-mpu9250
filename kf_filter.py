import numpy as np
import matplotlib.pyplot as plt
import sys

ROLL = './data/roll_data/roll_data.txt', './data/roll_data/kf_filter_data.txt'
PITCH = './data/pitch_data/pitch_data.txt', './data/pitch_data/kf_filter_data.txt'
DATA_SETS = [ROLL, PITCH]
DATA_SET_INDEX = sys.argv[1] if len(sys.argv) > 1 else 0
DATA_SET_INDEX = int(DATA_SET_INDEX)
FILENAMES = DATA_SETS[DATA_SET_INDEX]  

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

        if DATA_SET_INDEX == 0:
            self.Q = 5
            self.R = 125
        else:
            self.Q = 125
            self.R = 5

        self.measurement_noise_covar = np.eye(4) * self.R
        self.process_noise_covar = np.eye(4) * self.Q

        self.initial_uncertainty = np.eye(4)

        self.dt = 0.01  # 100 Hz
        
        self.read_sensor()

        self.filter()

        self.read_results()

        self.plot_results()

    def filter(self):

        self.process_noise_covar = np.eye(4) * self.Q
        self.measurement_noise_covar = np.eye(4) * self.R
        
        x_old = np.array([[1., 0., 0., 0.]]).T  # initial state
        P_old = self.initial_uncertainty  # initial uncertainty

        with open(FILENAMES[1], 'w') as f:
            f.write("measured_roll,measured_pitch,measured_yaw,filtered_roll,filtered_pitch,filtered_yaw\n")

        for i in range(self.total_samples):

            ax = self.imu_data['ax'][0][i]
            ay = self.imu_data['ay'][0][i]
            az = self.imu_data['az'][0][i]

            gx = self.imu_data['gx'][0][i]
            gy = self.imu_data['gy'][0][i]
            gz = self.imu_data['gz'][0][i]

            self.accel = np.array([[ax, ay, az]])
            self.gyro = np.array([[gx, gy, gz]])

            self.euler_attitude = self.roll_pitch(*self.accel[0])
            self.gyro = self.gyro_to_euler(*self.gyro[0], *self.euler_attitude[0][:2])

            gx, gy, gz = self.gyro[0]
            self.measured_quaternion = self.euler_to_quat(*self.euler_attitude[0])

            norm = np.linalg.norm(self.measured_quaternion)
            if norm > 1e-6:
                self.measured_quaternion = self.measured_quaternion / norm

            Un = np.array([[0, -gx, -gy, -gz],
                          [gx, 0, gz, -gy],
                          [gy, -gz, 0, gx],
                          [gz, gy, -gx, 0]])
            
            self.A = np.eye(4) + (0.5 * self.dt * Un)
            
            x_pred = self.A @ x_old
            P_pred = self.A @ P_old @ self.A.T + self.process_noise_covar

            K_gain = P_pred @ self.C.T @ np.linalg.inv(self.C @ P_pred @ self.C.T + self.measurement_noise_covar)

            x_new = x_pred + K_gain @ (self.measured_quaternion.T - self.C @ x_pred)
            P_new = (np.eye(4) - K_gain @ self.C) @ P_pred

            norm = np.linalg.norm(x_new)
            if norm > 1e-6:
                x_new = x_new / norm

            x_old = x_new
            P_old = P_new

            self.measured_attitude = self.quat_to_euler(*self.measured_quaternion[0])
            self.filtered_attitude = self.quat_to_euler(*x_new.T[0])

            self.append_results(self.measured_quaternion, self.filtered_attitude)


    def append_results(self, measured, filtered):
        try:
            with open(FILENAMES[1], 'a') as f:
                f.write(f"{measured[0,0]:<.3f},{measured[0,1]:<.3f},{measured[0,2]:<.3f},{filtered[0,0]:<.3f},{filtered[0,1]:<.3f},{filtered[0,2]:<.3f}\n")

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
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = np.sqrt(1 + 2 * (qw * qy - qx * qz))
        cosp = np.sqrt(1 - 2 * (qw * qy - qx * qz))
        pitch = 2 * np.arctan2(sinp, cosp) - np.pi / 2
        #pitch = np.arcsin(2 * (qw * qy - qx * qz))

        # yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([[roll, pitch, yaw]])
    
    def read_sensor(self):
        try:
            data = np.loadtxt(FILENAMES[0], delimiter=',', skiprows=1)
            self.imu_data = {k: data[:, i] for i, k in enumerate(['ax', 'ay', 'az', 'gx', 'gy', 'gz'])}
            self.imu_data = {k: np.array([v]) for k, v in self.imu_data.items()}

            self.total_samples = np.size(self.imu_data['ax'][0])

        except Exception as e:
            self.imu_data = None

    def read_results(self):
        try:
            data = np.loadtxt(FILENAMES[1], delimiter=',', skiprows=1)
            self.predicted_data = {k: data[:, i] for i, k in enumerate(['m_roll', 'm_pitch', 'm_yaw', 'f_roll', 'f_pitch', 'f_yaw'])}
            self.predicted_data = {k: np.array([v]) for k, v in self.predicted_data.items()}

        except Exception as e:
            self.predicted_data = None

    def plot_results(self, sample_size=None, plot_option=PLOT_OPTION):

        if sample_size is None:
            sample_size = self.total_samples - 1

        if plot_option:
            sample_size = sample_size + 1

            if ANGLE_CHOICE == 'deg':
                ylabel = ' (deg)'
                self.predicted_data['m_roll'] = self.predicted_data['m_roll'] * 180/np.pi
                self.predicted_data['m_pitch'] = self.predicted_data['m_pitch'] * 180/np.pi
                self.predicted_data['m_yaw'] = self.predicted_data['m_yaw'] * 180/np.pi
                self.predicted_data['f_roll'] = self.predicted_data['f_roll'] * 180/np.pi
                self.predicted_data['f_pitch'] = self.predicted_data['f_pitch'] * 180/np.pi
                self.predicted_data['f_yaw'] = self.predicted_data['f_yaw'] * 180/np.pi
            elif ANGLE_CHOICE == 'rad':
                ylabel = ' (rad)'

            plt.figure(figsize=(10, 8))

            plt.subplots_adjust(hspace=0.5)

            plt.subplot(2, 1, 1)
            plt.plot(np.arange(sample_size), self.predicted_data['m_roll'][0,:sample_size], label='measured_roll', color='r')

            plt.plot(np.arange(sample_size), self.predicted_data['f_roll'][0,:sample_size], label='filtered_roll', color='r', linestyle='dashed')

            plt.legend(loc='lower right')
            plt.ylabel('roll' + ylabel)
            plt.yticks(np.arange(-180, 181, 45))
            plt.ylim([-180, 180])
            plt.grid(which='both', linestyle='--', linewidth=0.5)
            plt.minorticks_on()

            plt.subplot(2, 1, 2)
            plt.plot(np.arange(sample_size), self.predicted_data['m_pitch'][0,:sample_size], label='measured_pitch', color='g')

            plt.plot(np.arange(sample_size), self.predicted_data['f_pitch'][0,:sample_size], label='filtered_pitch', color='g', linestyle='dashed')

            plt.legend(loc='lower right')
            plt.ylabel('pitch' + ylabel)
            plt.yticks(np.arange(-180, 181, 45))
            plt.ylim([-180, 180])
            plt.grid(which='both', linestyle='--', linewidth=0.5)
            plt.minorticks_on()

            plt.show()

if __name__ == '__main__':
    np.set_printoptions(precision=3, suppress=True)

    kf = KalmanFilter()
