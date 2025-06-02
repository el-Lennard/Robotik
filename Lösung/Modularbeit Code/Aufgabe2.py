import numpy as np
import matplotlib.pyplot as plt
import pickle
import robolib
import pandas as pd
from robo_communication import RoboCommunication
from robot_datapack_mapping import Keys
from Aufgabe1 import set_axes_equal_3d


# dh-parameters from UR-Sim for UR5
# /home/ur/ursim-current/.urcontrol/urcontrol.conf
ur5 = np.array([[np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0],
                [0.00000, -0.42500, -0.39225, 0.00000, 0.00000, 0.0000],
                [0.089159, 0.00000, 0.00000, 0.10915, 0.09465, 0.0823],
                [0, -np.pi / 2, 0, -np.pi / 2, 0, 0]]).T


def plan_traj(start_q, start_pose, target_pose, a, v, plot=False, csv=True):
    tcp_poses, tcp_vs, tcp_as = robolib.get_traj(start_pose, target_pose, a, v)

    qs = robolib.ik_traj_ur(ur5, tcp_poses, start_q)
    qds = np.array([robolib.q_dot_from_pose_dot(ur5, q, tcp_v) for q, tcp_v in zip(qs, tcp_vs)])
    # qdd_array =   # Ableiten nötig, oder Trajektorienplanung im Arbeitsraum (?)

    # # qs und qds als CSV speichern
    # np.savetxt('A2_Q_plan.csv', qs, qds, delimiter=',')

    # pd.DataFrame({'TOOL_VECTOR_TARGET': tcp_poses, 'TCP_SPEED_TARGET': tcp_vs, 'Q_TARGET': qs, 'QD_TARGET': qds}
    #              ).to_csv('A2_Q_plan.csv', index=False)
    if csv:
        combined_df = pd.concat([pd.DataFrame(tcp_poses), pd.DataFrame(tcp_vs), pd.DataFrame(qs),
                                 pd.DataFrame(qds)], ignore_index=True, axis=1)
        combined_df.columns = ['x', 'y', 'z', 'rx', 'ry', 'rz', 'vx', 'vy', 'vz', 'vrx', 'vry', 'vrz', 'q1', 'q2', 'q3',
                               'q4', 'q5', 'q6', 'qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6']
        combined_df.to_csv("A2_plan.csv", index=False)

    if plot:
        x = tcp_poses[:, 0]
        y = tcp_poses[:, 1]
        z = tcp_poses[:, 2]
        # start_pose_plan = (x[0], y[0], z[0])
        # print("start pose plan", start_pose_plan)
        # target_pose_plan = (x[-1], y[-1], z[-1])
        # print("target pose plan", target_pose_plan)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(x, y, z)
        set_axes_equal_3d(ax)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('planned')

    return np.array([tcp_poses, tcp_vs, qs, qds])


def analyze_recording(pkl_name, plt_name, actual=False, plot=False, csv=True):
    with open(f'{pkl_name}.pkl', 'rb') as f:
        record = pickle.load(f)

    # record = record[2:]
    # pickle.dump(record, open(f'{pkl_name}.pkl', 'wb'))
    if actual:
        Ks = [Keys.TOOL_VECTOR_ACTUAL, Keys.TCP_SPEED_ACTUAL, Keys.Q_ACTUAL, Keys.QD_ACTUAL]
    else:
        Ks = [Keys.TOOL_VECTOR_TARGET, Keys.TCP_SPEED_TARGET, Keys.Q_TARGET, Keys.QD_TARGET,]

    data = RoboCommunication.unpack_recording(record, Ks)

    if csv:
        combined_df = pd.DataFrame()
        for k in Ks:
            combined_df = pd.concat([combined_df, pd.DataFrame(data[k.name])], axis=1)
            # np.savetxt(f'A2_{k.name}_{plt_name}.csv', np.array(data[k.name]), delimiter=',')
        combined_df.columns = ['x', 'y', 'z', 'rx', 'ry', 'rz', 'vx', 'vy', 'vz', 'vrx', 'vry', 'vrz', 'q1', 'q2', 'q3',
                               'q4', 'q5', 'q6', 'qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6']
        combined_df.to_csv(f"A2_{plt_name}.csv", index=False)

    if plot:
        RoboCommunication.plot_joint_recording_together(data, qdd_act=False, title='Real Robot Movement', time=False,
                                                        degree=True, show_plot=True)
        if actual:
            tcp = Keys.TOOL_VECTOR_ACTUAL
        else:
            tcp = Keys.TOOL_VECTOR_TARGET
        x, y, z, *_ = np.array(data[tcp]).T

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x, y, z)
        set_axes_equal_3d(ax)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(plt_name)
    return np.array([data[k.name] for k in Ks])


def plot_together_subplots(plan, servo_j, move_l, time=True, actual=False):
    servo_j = servo_j.copy()
    move_l = move_l.copy()
    plan = plan.copy()
    titles = ['TCP Poses', 'TCP Velocities', 'Joint Angles', 'Joint Velocities']
    file_names = ['tcp_poses', 'tcp_velocities', 'joint_angles', 'joint_velocities']
    legend = ['Plan', 'ServoJ', 'MoveL']
    ax_titles_tcp = ['x', 'y', 'z', 'rx', 'ry', 'rz']
    ax_titles_q = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6']
    for i in range(4):
        fig, ax = plt.subplots(6, sharex=True, figsize=(10, 10))
        for j in range(6):
            if i in (0, 1):     # tcp
                ax[j].set_title(ax_titles_tcp[j])
                if i == 0:
                    ax[j].set_ylabel('m' if j < 3 else 'rad')
                else:
                    ax[j].set_ylabel('m/s' if j < 3 else 'rad/s')
            else:     # q
                plan[i, :, j] = plan[i, :, j] * 180 / np.pi
                servo_j[i, :, j] = servo_j[i, :, j] * 180 / np.pi
                move_l[i, :, j] = move_l[i, :, j] * 180 / np.pi
                ax[j].set_title(ax_titles_q[j])
                if i == 2:
                    ax[j].set_ylabel('deg')
                else:
                    ax[j].set_ylabel('deg/s')
            servo_x = np.arange(len(servo_j[i, :, j]))
            move_x = np.arange(len(move_l[i, :, j]))
            plan_offset = 4
            plan_x = np.arange(len(plan[i, :, j]))+plan_offset
            if time:
                servo_x = servo_x * 0.008
                move_x = move_x * 0.008
                plan_x = plan_x * 0.008
            ax[j].plot(plan_x, plan[i, :, j])
            ax[j].plot(servo_x, servo_j[i, :, j])
            ax[j].plot(move_x, move_l[i, :, j])
            if j == 5:
                if time:
                    ax[j].set_xlabel('time [s]')
                else:
                    ax[j].set_xlabel('samples')
            # ax[j].legend(['ServoJ', 'MoveL'])
        same_y_scale(ax)
        fig.suptitle(f'{titles[i]} Actual' if actual else titles[i], y=0.94)  # Adjusted y parameter for suptitle
        fig.legend(legend, loc='lower center', ncol=len(legend), bbox_to_anchor=(0.5, 0.02))
        plt.savefig(f'2_{file_names[i]}_actual' if actual else f'2_{file_names[i]}', dpi=300, bbox_inches='tight')
        # plt.tight_layout(pad=2)


def same_y_scale(axs):
    lims = [ax.get_ylim() for ax in axs]
    max_range = max([lim[1] - lim[0] for lim in lims])
    y_middle = np.mean(lims, axis=1)
    for i in range(6):
        axs[i].set_ylim([y_middle[i] - max_range / 2, y_middle[i] + max_range / 2])


def main():
    start_q = np.array([30, -60, 90, 0, 90, 0])/180*np.pi
    start_pose = robolib.T_2_rotvec(robolib.fk_ur(ur5, start_q))
    print('start pose x, y, z [m] and rx, ry, rz [rad] ', start_pose)
    target_pose = start_pose + np.array([0.05, 0, 0, 0, 0, 0])  # 0.05 ist delta
    print('target pose', target_pose)
    target_q = robolib.ik_ur(ur5, start_pose, target_pose) * 180 / np.pi
    print('target q', target_q)
    a = 0.8
    v = 0.1
    plan = plan_traj(start_q, start_pose, target_pose, a, v, plot=True)
    movel = analyze_recording(pkl_name='movel_cut', plt_name='movel', actual=False)
    servoj = analyze_recording(pkl_name='servoj_cut', plt_name='servoj', actual=False)
    plot_together_subplots(plan, servoj, movel)
    movel_act = analyze_recording(pkl_name='movel_cut', plt_name='movel_act', actual=True)
    servoj_act = analyze_recording(pkl_name='servoj_cut', plt_name='servoj_act', actual=True)
    plot_together_subplots(plan, servoj_act, movel_act, actual=True)
    plt.show()


if __name__ == '__main__':
    main()
