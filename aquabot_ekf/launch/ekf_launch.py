from simple_launch import SimpleLauncher
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)
    sl.declare_arg('rviz', True)

    with sl.group(if_arg = 'rviz'):
        sl.rviz(sl.find('aquabot_ekf', 'ekf.rviz'))

    for link in ('base_link', 'imu_wamv_link', 'gps_wamv_link', 'receiver'):
        sl.node('tf2_ros', 'static_transform_publisher', name='static_'+link,
                arguments = ['--frame-id', 'wamv/'+link, '--child-frame-id', 'wamv/wamv/'+link])
        

    sl.node('aquabot_ekf','gps2pose')

    # run an EKF for wamv
    sl.node('robot_localization', 'ekf_node', name = 'ekf',
            parameters = [sl.find('aquabot_ekf', 'ekf.yaml')],
            namespace = 'wamv',
            remappings = {'odometry/filtered': 'odom'},
            output='screen')

    # run EKFs for friends
    #ekf_friend = sl.find('aquabot_ekf', 'ekf_friend.yaml')
    #for friend in ('friend0', 'friend1'):

        #configured_params = RewrittenYaml(source_file = ekf_friend,
                                          #param_rewrites={'base_link_frame': friend},
                                          #convert_types=True)

        #sl.node('robot_localization', 'ekf_node', name = 'ekf',
                #parameters = [configured_params],
                #namespace = friend,
                #remappings = {'odometry/filtered': 'odom'})

    return sl.launch_description()
