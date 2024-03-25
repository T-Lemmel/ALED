from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)
    sl.declare_arg('rviz', True)

    # transfert GPS to actual poses
    sl.node('aquabot_ekf', 'gps2pose',
            output='screen')

    with sl.group(if_arg = 'rviz'):
        sl.rviz(sl.find('aquabot_ekf', 'ekf.rviz'))

    return sl.launch_description()
