import time
import numpy as np
import pandas as pd
from robo_communication import RoboCommunication
import robolib
from robot_datapack_mapping import Keys
import pickle
import matplotlib.pyplot as plt


def a_trajectory_of_joints_plan(start_q, target_q, v, a, plot=False):
    """Theoretical values of python implementation"""
    qs, qds, qdds = robolib.get_traj(start_q, target_q, a, v, t_step=8e-3)
    # Am Anfang und am Ende der Trajektorie die Geschwindigkeit und Beschleunigung mit 0 einfügen
    offset = 10
    # start_q, und zeroes so oft wie offset einfügen
    start_q = np.tile(start_q, (offset, 1))
    zeroes = np.tile(np.zeros(6), (offset, 1))
    qs = np.vstack((start_q, qs, target_q))
    qds = np.vstack((zeroes, qds, np.zeros(6)))
    qdds = np.vstack((zeroes, qdds, np.zeros(6)))
    data = {Keys.Q_TARGET.name: list(qs), Keys.QD_TARGET.name: list(qds), Keys.QDD_TARGET.name: list(qdds)}
    if plot:
        RoboCommunication.plot_joint_recording_together(data, title='Python Implementation', degree=True, show_plot=False)
    # TODO: Zeit für x-Achse berechnen und plotten
    return data


def record_raw_data(start_q, target_q, v, a):
    robo = RoboCommunication('192.168.52.128')
    robo.movej_rad(start_q)
    time.sleep(7)
    robo.movej_rad(target_q, a=a, v=v)
    # record = robo.record_movement()
    record = robo.record_t(5)
    # pickle record
    # filename current date time
    filename = time.strftime("%Y%m%d-%H%M%S-Prakt1")
    with open(f'{filename}.pkl', 'wb') as f:
        pickle.dump(record, f)


def a_trajectory_of_joints_robot(pkl_name, actual=False, plot=False, csv=True):
    """Values of real robot movement"""
    # read pickle file
    with open(f'{pkl_name}.pkl', 'rb') as f:
        record = pickle.load(f)

    record = record[878:1132]

    if actual:
        Ks = [Keys.TOOL_VECTOR_ACTUAL, Keys.TCP_SPEED_ACTUAL, Keys.Q_ACTUAL, Keys.QD_ACTUAL, Keys.QDD_TARGET]
    else:
        Ks = [Keys.TOOL_VECTOR_TARGET, Keys.TCP_SPEED_TARGET, Keys.Q_TARGET, Keys.QD_TARGET, Keys.QDD_TARGET]

    data = RoboCommunication.unpack_recording(record, Ks)

    if actual:
        # qdd_actual berechnen
        qd_array = data[Keys.QD_ACTUAL.name]
        time_array = data[Keys.TIME.name]
        time_diff_array = np.array([time_array[i] - time_array[i - 1] for i in range(1, len(time_array))])
        # data[Keys.QDD_TARGET.name] = np.array([qd_array[i] - qd_array[i - 1] for i in range(1, len(qd_array))]) / 8e-3
        data[Keys.QDD_TARGET.name] = np.array([qd_array[i] - qd_array[i - 1] for i in range(1, len(qd_array))]) / time_diff_array
        # glätten von qdd über 8 Punkte durch average der letzten 8 Punkte
        # data[Keys.QDD_TARGET.name] = np.array([np.convolve(data[Keys.QDD_TARGET.name][:, i], np.ones(8) / 8, mode='same')
        #                                        for i in range(6)]).T        # T: reshape to (n, 6)
        data[Keys.Q_ACTUAL.name] = data[Keys.Q_ACTUAL.name][1:]
        data[Keys.QD_ACTUAL.name] = data[Keys.QD_ACTUAL.name][1:]
        data[Keys.TIME.name] = data[Keys.TIME.name][1:]

    if csv:
        combined_df = pd.DataFrame()
        for k in Ks:
            combined_df = pd.concat([combined_df, pd.DataFrame(data[k.name])], axis=1)
            # np.savetxt(f'A2_{k.name}_{plt_name}.csv', np.array(data[k.name]), delimiter=',')
        combined_df.columns = ['x', 'y', 'z', 'rx', 'ry', 'rz', 'vx', 'vy', 'vz', 'vrx', 'vry', 'vrz', 'q1', 'q2',
                               'q3', 'q4', 'q5', 'q6', 'qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6',
                               'qdd1', 'qdd2', 'qdd3', 'qdd4', 'qdd5', 'qdd6']
        filename = f'A1_robot_actual.csv' if actual else f'A1_robot_target.csv'
        combined_df.to_csv(filename, index=False)
    if plot:
        RoboCommunication.plot_joint_recording_together(data, qdd_act=True, title='Real Robot Movement', degree=True, show_plot=False, time=False)

    return data


def b_trajectory_of_tcp_pose_plan(csv=True):    # TODO: Angaben in main
    start_q = np.array([30, -60, 90, 0, 90, 0]) / 180 * np.pi
    delta_q = np.array([0, -40, -10, 0, 0, 0]) / 180 * np.pi  # axis 2-3 movement of delta
    a = 0.8  # given acceleration for movej
    v = 0.6  # given velocity for movej
    target_q = start_q + delta_q
    qs, qds, qdds = robolib.get_traj(start_q, target_q, a, v)
    tcp_poses = [robolib.T_2_rotvec(robolib.fk_ur(ur5, q)) for q in qs]
    tcp_vs = [robolib.pose_dot_from_q_dot(ur5, q, qd) for q, qd in zip(qs, qds)]

    if csv:
        combined_df = pd.concat([pd.DataFrame(tcp_poses), pd.DataFrame(tcp_vs), pd.DataFrame(qs),
                                 pd.DataFrame(qds), pd.DataFrame(qdds)], ignore_index=True, axis=1)
        combined_df.columns = ['x', 'y', 'z', 'rx', 'ry', 'rz', 'vx', 'vy', 'vz', 'vrx', 'vry', 'vrz', 'q1', 'q2',
                               'q3', 'q4', 'q5', 'q6', 'qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6',
                               'qdd1', 'qdd2', 'qdd3', 'qdd4', 'qdd5', 'qdd6']
        combined_df.to_csv("A1_plan.csv", index=False)

    # Extract the x, y, and z coordinates
    x, y, z, *_ = np.array(tcp_poses).T

    plot_3d_2d(x, y, z, 'Python Implementation')


def b_trajectory_of_tcp_pose_real(pkl_name):
    with open(f'{pkl_name}.pkl', 'rb') as f:
        record = pickle.load(f)

    record = record[878:1132]

    data = RoboCommunication.unpack_recording(record, [Keys.TOOL_VECTOR_ACTUAL, Keys.TCP_SPEED_ACTUAL])

    # Extract the x, y, and z coordinates
    x, y, z, *_ = np.array(data[Keys.TOOL_VECTOR_ACTUAL.name]).T

    plot_3d_2d(x, y, z, 'Real Robot Movement')


def plot_3d_2d(x, y, z, name):
    # draw tcp pose in 3D over time
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(x, y, z)
    set_axes_equal_3d(ax)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(name)

    # Ebene im 3d Plot einzeichne

    # Richtungspfeil an Trajektorie
    # ax.quiver(x[0], y[0], z[0], x[len(x)//4] - x[0], y[len(x)//4] - y[0], z[len(x)//4] - z[0], color='r')

    # Optional orient the plot:
    # ax.view_init(elev=90, azim=270)

    # draw tcp pose in 2D over time
    # Define the plane using 3 points
    middle = len(x) // 2
    p1 = np.array([x[0], y[0], z[0]])
    p2 = np.array([x[middle], y[middle], z[middle]])
    p3 = np.array([x[-1], y[-1], z[-1]])

    # Calculate two direction vectors in the plane
    v1 = p2 - p1
    v2 = p3 - p1

    # # Heßsche Normalform der Ebene
    # normal = np.cross(v1, v2)
    # normal = normal / np.linalg.norm(normal)  # Normalisierung
    #
    # print(f'Die Ebenengleichung ist {normal}*(p-{p1}) = 0')
    #
    # A, B, C = normal
    # D = -np.dot(normal, p1)
    # print(f"Die Ebenengleichung ist: {A}x + {B}y + {C}z + {D} = 0")
    #
    # # Ebene plotten
    # xx, yy = np.meshgrid(np.arange(min(x), max(x), 0.001), np.arange(min(y), max(y), 0.001))
    # zz = (-normal[0] * xx - normal[1] * yy - D) * 1. / normal[2]
    # ax.plot_surface(xx, yy, zz, color='lightgray', alpha=0.2, label='Ebene')
    #
    # # z skalierung
    # ax.set_zlim(zmin=min(z), zmax=max(z))

    # Normalize the direction vectors
    v1_norm = v1 / np.linalg.norm(v1)
    # n = np.cross(v1_norm, v2)
    # v_orth_v1 = np.cross(n, v1_norm)
    # v2_norm = v_orth_v1 / np.linalg.norm(v_orth_v1)
    v2_proj = v2 - np.dot(v2, v1_norm) * v1_norm  # Project v2 orthogonal to v1_norm onto the plane
    v2_norm = v2_proj / np.linalg.norm(v2_proj)
    print(np.arccos(np.dot(v2_norm, v1_norm))/np.pi*180)

    # # basisvektoren in 3d einzeichnen
    ax.quiver(*p1, *v1_norm, length=0.1, color='r', label='Basis Vector 2')
    ax.quiver(*p1, *v2_norm, length=0.1, color='g', label='Basis Vector 1')

    # print(np.dot(v1_norm, v2_norm))  # should be 0
    # print(np.dot(v1_norm, v2_proj))  # should be 0

    # Create the basis of the plane (2x3 matrix)
    basis = np.stack([v2_norm, v1_norm]).T

    # Subtract the offset (p1) from all points
    points_3D = np.array([x, y, z]).T - p1

    # Transform the 3D points to the 2D plane
    points2D = np.dot(points_3D, basis)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(points2D[:, 0], points2D[:, 1])
    set_axes_equal_2d(ax)
    plt.xlabel(f'Basis Vector 1: {basis[:, 0]}')
    plt.ylabel(f'Basis Vector 2: {basis[:, 1]}')
    plt.title(name)


def set_axes_equal_3d(ax: plt.Axes):
    """
    Setze alle drei Achsen auf die gleiche Schrittweite. Dadurch entsteht ein korrekter visueller Eindruck im Raum.
    """
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])

    max_range = max(x_range, y_range, z_range)

    x_middle = np.mean(x_limits)
    y_middle = np.mean(y_limits)
    z_middle = np.mean(z_limits)

    ax.set_xlim3d([x_middle - max_range/2, x_middle + max_range/2])
    ax.set_ylim3d([y_middle - max_range/2, y_middle + max_range/2])
    ax.set_zlim3d([z_middle - max_range/2, z_middle + max_range/2])


def set_axes_equal_2d(ax: plt.Axes):
    x_limits = ax.get_xlim()
    y_limits = ax.get_ylim()

    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])

    # Den größten Bereich finden
    max_range = max(x_range, y_range)

    x_middle = np.mean(x_limits)
    y_middle = np.mean(y_limits)

    # Achsengrenzen setzen
    ax.set_xlim([x_middle - max_range / 2, x_middle + max_range / 2])
    ax.set_ylim([y_middle - max_range / 2, y_middle + max_range / 2])


def b_angle_z_axis_to_x_y_plane(data_list, labels):
    plt.figure()
    for i, data in enumerate(data_list):
        if Keys.Q_ACTUAL.name in data:
            angles_z_2_xy = np.array([(np.pi / 2 - np.arccos(robolib.fk_ur(ur5, angle)[2, 2])) * 180 / np.pi for angle in
                                      data[Keys.Q_ACTUAL.name]])
        else:
            angles_z_2_xy = np.array([(np.pi / 2 - np.arccos(robolib.fk_ur(ur5, angle)[2, 2])) * 180 / np.pi for angle in
                                      data[Keys.Q_TARGET.name]])
        t = np.arange(len(angles_z_2_xy)) * 8e-3
        plt.plot(t, angles_z_2_xy, label=labels[i])
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [deg] TCP z-axis to x-y-plane of base system')
    plt.legend()
    plt.savefig('1b_angle_z_axis', dpi=300, bbox_inches='tight')


def plot_joint_recording_complete(record, degree=False, time=True, actual=False, fig=None, axs=None, legend=None):
    """
    Plots angle, velocity and acceleration of all joints seperately.
    """
    joint_names = ['Base', 'Shoulder', 'Elbow', 'Wrist1', 'Wrist2', 'Wrist3']
    value_names = ['Angle', 'Velocity', 'Acceleration']
    if actual:
        values = [Keys.Q_ACTUAL, Keys.QD_ACTUAL, Keys.QDD_TARGET]
    else:
        values = [Keys.Q_TARGET, Keys.QD_TARGET, Keys.QDD_TARGET]

    if axs is None:
        fig, axs = plt.subplots(6, 3, sharex=True, figsize=(20, 20))  # 6 rows for joints, 3 columns for values

    for i, ax_row in enumerate(axs):  # iterate over rows (joints)
        for j, ax in enumerate(ax_row):  # iterate over columns (values)
            value = values[j]
            x_axis = np.arange(0, len(record[value.name]) * 8e-3, 8e-3) if time else range(len(record[value.name]))
            ax.plot(x_axis, [r[i]
                             if not degree
                             else r[i] / np.pi * 180
                             for r in record[value.name]])
            # ax.set_title(f'{joint_names[i]} - {value.name}')
            ax.set_title(f'{joint_names[i]} {value_names[j]}')
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
            # if legend is not None:
            #     ax.legend(legend)
    if legend is not None:
        fig.legend(legend, loc='lower center', ncol=len(legend), bbox_to_anchor=(0.5, 0.05))
        plt.savefig(f'1_joint_angles_velocities_accelerations', dpi=300, bbox_inches='tight')
    return fig, axs


def plot_joint_recording_together(record, degree=False, time=True, actual=False, qdd_act=False, fig=None, axs=None, joint=None, legend=None):
    """
    Plots velocity, acceleration and angle joint(s) together.
    """
    joint_names = ['Base', 'Shoulder', 'Elbow', 'Wrist1', 'Wrist2', 'Wrist3']
    if actual:
        values = [Keys.Q_ACTUAL, Keys.QD_ACTUAL, Keys.QDD_TARGET]  # always use these values
    else:
        values = [Keys.Q_TARGET, Keys.QD_TARGET, Keys.QDD_TARGET]

    if axs is None:
        fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)  # 3 rows for values

    for j, ax in enumerate(axs):  # iterate over rows (values)
        value = values[j]
        if joint is not None:
            start = joint - 1
            stop = joint
        else:
            start = 0
            stop = 6
        for i in range(start, stop):  # iterate over joints
            x_axis = np.arange(0, len(record[value.name]) * 8e-3, 8e-3) if time else range(len(record[value.name]))
            ax.plot(x_axis, [r[i]
                             if not degree
                             else r[i] / np.pi * 180
                             for r in record[value.name]], label=joint_names[i])
            ax.set_title(f'{value.name.split("_")[0]}')
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
            # ax.legend()  # show legend
    if joint is not None:
        fig.suptitle(joint_names[joint-1], y=0.94)
    if legend is not None:
        fig.legend(legend, loc='lower center', ncol=len(legend), bbox_to_anchor=(0.5, 0.02))
        plt.savefig(f'1_{joint_names[joint-1].lower()}', dpi=300, bbox_inches='tight')
    return fig, axs


def main():
    start_q = np.array([30, -60, 90, 0, 90, 0]) / 180 * np.pi
    delta_q = np.array([0, -40, -10, 0, 0, 0]) / 180 * np.pi  # axis 2-3-movement of delta
    a = 0.8  # given acceleration for movej
    v = 0.6  # given velocity for movej
    target_q = start_q + delta_q

    # record_raw_data(start_q, target_q, v, a)
    record = 'movej'
    plan = a_trajectory_of_joints_plan(start_q, target_q, v, a)
    robot_target = a_trajectory_of_joints_robot(record)
    robot_actual = a_trajectory_of_joints_robot(record, actual=True)
    fig, axs = plot_joint_recording_complete(plan)
    plot_joint_recording_complete(robot_target, axs=axs)
    legend = ['Python Implementation', 'Robot Target', 'Robot Actual']
    plot_joint_recording_complete(robot_actual, fig=fig, axs=axs, legend=legend, actual=True)

    # shoulder
    fig, ax = plot_joint_recording_together(plan, joint=2, degree=True)
    plot_joint_recording_together(robot_target, fig=fig, axs=ax, joint=2, degree=True)
    plot_joint_recording_together(robot_actual, fig=fig, axs=ax, joint=2, degree=True, actual=True, legend=legend)

    # elbow
    fig, ax = plot_joint_recording_together(plan, joint=3, degree=True)
    plot_joint_recording_together(robot_target, fig=fig, axs=ax, joint=3, degree=True)
    plot_joint_recording_together(robot_actual, fig=fig, axs=ax, joint=3, degree=True, actual=True, legend=legend)

    b_trajectory_of_tcp_pose_plan()
    b_trajectory_of_tcp_pose_real(record)

    b_angle_z_axis_to_x_y_plane([plan, robot_target, robot_actual],
                                ['Python Implementation', 'Robot Target', 'Robot Actual'])
    plt.show()


if __name__ == '__main__':
    ur5 = np.array([[np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0],
                    [0.00000, -0.42500, -0.39225, 0.00000, 0.00000, 0.0000],
                    [0.089159, 0.00000, 0.00000, 0.10915, 0.09465, 0.0823],
                    [0, -np.pi / 2, 0, -np.pi / 2, 0, 0]]).T
    main()
