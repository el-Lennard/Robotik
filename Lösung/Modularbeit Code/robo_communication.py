import time
import numpy as np
import socket
import struct
import matplotlib.pyplot as plt
import robot_datapack_mapping as mapping
from robot_datapack_mapping import Keys


class RoboCommunication:
    def __init__(self, ip, datapack_size=1120) -> None:
        self.datapack_size = datapack_size   # Lab: 1116! Sim: 1220!
        TX_PORT = 30002     # Send
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tx.connect((ip, TX_PORT))

        self.ip = ip
        RX_PORT = 30013     # Receive
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.rx.connect((ip, RX_PORT))
        # self.rx.settimeout(5.0)  # Set a timeout of 5 seconds

    def recv(self):
        return self.rx.recv(self.datapack_size)

    def recv_unpack(self, names: list[Keys] | Keys):
        """
        Receives a package and unpacks it according to value name(s).

        Parameters:
            names (list[Keys] | Keys): Value name(s).
        Returns:
            np.ndarray: Unpacked package.
        """
        r = self.rx.recv(self.datapack_size)
        return self.unpack(r, names)

    @staticmethod
    def unpack(r: bytes, names: list[Keys] | Keys):
        """
        Unpacks a package according to value name(s).

        Parameters:
            r (bytes): Package.
            names (list[Keys] | Keys): Value name(s).
        Returns:
            np.ndarray: Unpacked package.
        """
        if isinstance(names, list):
            return np.array(mapping.unpack_batch(r, names))
        elif isinstance(names, Keys):
            return np.array(mapping.unpack(r, names))

    @staticmethod
    def unpack_all(r: bytes):
        return mapping.unpack_all(r)

    @staticmethod
    def unpack_recording(rs, values: Keys | list[Keys]):
        values = values.copy()
        if not Keys.TIME in values:
            values.insert(0, Keys.TIME)
        received_values = {val.name: [] for val in values}
        for r in rs:
            for val in values:
                received_values[val.name].append(RoboCommunication.unpack(r, val))
        return received_values

    # methode um die aktuellen Gelenkwinkel in deg zu bekommen
    def get_joints_deg(self):
        rec = self.recv_unpack(Keys.Q_ACTUAL)
        return np.array(rec) * 180 / np.pi

    def movej_rad(self, q: list[float | int] | str | np.ndarray, a: float = None, v: float = None):
        if isinstance(q, np.ndarray):
            q = q.tolist()
        q = str(q)
        if a:
            q += f', a={a}'
        if v:
            q += f', v={v}'
        self.tx.send(f"movej({q})\n".encode('ascii'))

    def textmovej_deg(self, q: list[float | int] | str | np.ndarray, a: float = None, v: float = None):
        if isinstance(q, str):
            q = [float(i) for i in q.strip('[]').split(',')]
        q = np.array(q) / 180 * np.pi     # convert to rad    # list so that string repr has commas
        self.movej_rad(q, a, v)

    def record_t(self, t):
        """
        Records the values for a certain time.

        Parameters:
            t (float): Time in seconds.
        """
        print('record t')
        rs = []
        amount_of_points = int(np.ceil(t*125))    # 125 Hz
        for _ in range(amount_of_points):
            rs.append(self.rx.recv(self.datapack_size))
            time.sleep(0.005)
        return rs

    def record_movement(self):
        # getestet: funktioniert auch schnell genug, sodass 8ms eingehalten werden
        print('record movement')
        tolerance = 1e-2
        max_wait_time = 10  # s
        amount_of_points = int(np.ceil(max_wait_time*125))
        # wait until movement starts
        print(f'wait for robot to move (max. {max_wait_time} s)')
        for i in range(amount_of_points):
            r = self.rx.recv(self.datapack_size)
            if any([abs(v) > tolerance for v in self.unpack(r, Keys.QD_ACTUAL)]):
                break
        # record movement
        rs = []
        print(f'recording movement (max. {max_wait_time} s)')
        n = 0
        for i in range(amount_of_points):
            r = self.rx.recv(self.datapack_size)
            rs.append(r)
            if all([abs(v) < tolerance for v in self.unpack(r, Keys.QD_ACTUAL)]):
                n += 1  # avoid break at pass through 0
            if n > 10:
                break   # only break if all joints stop at 0 for some time
        return rs

    @staticmethod
    def plot_joint_recording(record, value: Keys | list[Keys], degree=False, show_plot=True):
        """
        Plots recorded joint velocities or accelerations of the robot.
        """
        joint_names = ['base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3']
        fig, axs = plt.subplots(3, 2, sharex=True)
        # title for plot window
        fig.suptitle(value.name)
        for i, ax in enumerate(axs.flat):  # subplots for each joint
            x_axis = record[Keys.TIME.name]-record[Keys.TIME.name][0] \
                if Keys.TIME.name in record \
                else range(len(record[value.name]))
            ax.plot(x_axis, [r[i]
                             if not degree
                             else r[i]/np.pi*180
                             for r in record[value.name]])
            ax.set_title(joint_names[i])
            # x-axis label
            if i >= 4:
                if Keys.TIME.name in record:
                    ax.set_xlabel('time [s]')
                else:
                    ax.set_xlabel('data point')
            if i % 2 == 0:
                if value == Keys.Q_ACTUAL:
                    ax.set_ylabel('rad') if not degree else ax.set_ylabel('deg')
                elif value == Keys.QD_ACTUAL:
                    ax.set_ylabel('rad/s') if not degree else ax.set_ylabel('deg/s')
                elif value == Keys.QDD_TARGET:
                    ax.set_ylabel('rad/s^2') if not degree else ax.set_ylabel('deg/s^2')
        plt.tight_layout()
        if show_plot:
            plt.show()

    @staticmethod
    def plot_joint_recording_complete(record, degree=False, show_plot=True, time=False, actual=False, axs=None):
        """
        Plots angle, velocity and acceleration of all joints seperately.
        """
        joint_names = ['base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3']
        value_names = ['Q', 'QD', 'QDD']
        if actual:
            values = [Keys.Q_ACTUAL, Keys.QD_ACTUAL, Keys.QDD_TARGET]  # always use these values
        else:
            values = [Keys.Q_TARGET, Keys.QD_TARGET, Keys.QDD_TARGET]

        if axs is None:
            fig, axs = plt.subplots(6, 3, sharex=True)  # 6 rows for joints, 3 columns for values

        for i, ax_row in enumerate(axs):  # iterate over rows (joints)
            for j, ax in enumerate(ax_row):  # iterate over columns (values)
                value = values[j]
                x_axis = record[Keys.TIME.name] - record[Keys.TIME.name][0] \
                    if Keys.TIME.name in record and time \
                    else range(len(record[value.name]))
                ax.plot(x_axis, [r[i]
                                 if not degree
                                 else r[i] / np.pi * 180
                                 for r in record[value.name]])
                # ax.set_title(f'{joint_names[i]} - {value.name}')
                ax.set_title(f'{joint_names[i]} - {value_names[j]}')
                # x-axis label
                if i == 5:  # last row
                    if Keys.TIME.name in record and time:
                        ax.set_xlabel('time [s]')
                    else:
                        ax.set_xlabel('data point')
                # y-axis label
                if value == Keys.Q_ACTUAL:
                    ax.set_ylabel('rad') if not degree else ax.set_ylabel('deg')
                elif value == Keys.QD_ACTUAL:
                    ax.set_ylabel('rad/s') if not degree else ax.set_ylabel('deg/s')
                elif value == Keys.QDD_TARGET:
                    ax.set_ylabel('rad/s^2') if not degree else ax.set_ylabel('deg/s^2')
        plt.tight_layout(pad=10.0)
        if show_plot:
            plt.show()
        return axs

    @staticmethod
    def plot_joint_recording_together(record, title='', degree=False, show_plot=True, time=True, actual=False, qdd_act=False, axs=None):
        """
        Plots velocity, acceleration and angle joint(s) together.
        """
        joint_names = ['base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3']
        if actual:
            values = [Keys.Q_ACTUAL, Keys.QD_ACTUAL, Keys.QDD_TARGET]  # always use these values
        else:
            values = [Keys.Q_TARGET, Keys.QD_TARGET, Keys.QDD_TARGET]

        if not axs:
            fig, axs = plt.subplots(3, 1, figsize=(10, 15), sharex=True)  # 3 rows for values
            fig.suptitle(title)

        for j, ax in enumerate(axs):  # iterate over rows (values)
            value = values[j]
            for i in range(6):  # iterate over joints
                x_axis = record[Keys.TIME.name] - record[Keys.TIME.name][0] \
                    if Keys.TIME.name in record and time \
                    else range(len(record[value.name]))
                ax.plot(x_axis, [r[i]
                                 if not degree
                                 else r[i] / np.pi * 180
                                 for r in record[value.name]], label=joint_names[i])
                ax.set_title(f'{value.name}')
                if qdd_act and value == Keys.QDD_TARGET:
                    ax.set_title('QDD_ACTUAL')
                # x-axis label
                if j == 2:  # last row
                    if Keys.TIME.name in record and time:
                        ax.set_xlabel('time [s]')
                    else:
                        ax.set_xlabel('data point')
                # y-axis label
                if value == Keys.Q_ACTUAL:
                    ax.set_ylabel('rad') if not degree else ax.set_ylabel('deg')
                elif value == Keys.QD_ACTUAL:
                    ax.set_ylabel('rad/s') if not degree else ax.set_ylabel('deg/s')
                elif value == Keys.QDD_TARGET:
                    ax.set_ylabel('rad/s^2') if not degree else ax.set_ylabel('deg/s^2')
                ax.legend()  # show legend
        plt.tight_layout(pad=3.0)
        if show_plot:
            plt.show()
        return axs


if __name__ == '__main__':
    ip = "192.168.52.128"
    robo = RoboCommunication(ip)
    test = robo.recv_unpack(Keys.Q_ACTUAL)
    print(test)
    test = robo.recv_unpack([Keys.Q_ACTUAL, Keys.TCP_SPEED_ACTUAL])
    print(test)
    robo.movej_deg([90, 0, 0, 0, 0, 0])
    time.sleep(6)
    robo.movej_rad(f'[0, {-np.pi/2}, 0, 0, 0, 0]')
    record = robo.record_t(6)
    robo.movej_deg('[0, 0, 0, 0, 0, 0]')
    record2 = robo.record_movement()
    print(robo.get_joints_deg())

    record = robo.unpack_recording(record, [Keys.Q_ACTUAL, Keys.QD_ACTUAL, Keys.QDD_TARGET])
    record2 = robo.unpack_recording(record2, [Keys.Q_ACTUAL, Keys.QD_ACTUAL, Keys.QDD_TARGET])

    robo.plot_joint_recording(record, Keys.Q_ACTUAL, degree=True)
    RoboCommunication.plot_joint_recording_together(record, degree=True)
    robo.plot_joint_recording(record, Keys.QD_ACTUAL, degree=True)
    robo.plot_joint_recording(record, Keys.QDD_TARGET, degree=True)
    robo.plot_joint_recording(record2, Keys.Q_ACTUAL, degree=True)
    robo.plot_joint_recording(record2, Keys.QD_ACTUAL, degree=True)
    robo.plot_joint_recording(record2, Keys.QDD_TARGET, degree=True)
