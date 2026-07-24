from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ROS_DOMAIN_ID는 launch 파일에서 강제로 지정하지 않습니다.
        # Doosan driver와 같은 터미널 환경값을 상속해야 서로 통신할 수 있습니다.
        # 실행 전 필요하면: export ROS_DOMAIN_ID=5

        # setup.py에서 console_scripts 이름을 'bridge'로 등록한 현재 구조 기준입니다.
        Node(
            package='cobotender',
            executable='bridge',
            output='screen',
        ),

        # bartender.py는 내부에서 dsr01 namespace의 두 ROS node를 직접 생성합니다.
        # 여기서 name=... 을 주면 launch가 __node remap을 걸어 내부 node 이름이 꼬일 수 있으므로 생략합니다.
        Node(
            package='cobotender',
            executable='bartender',
            output='screen',
        ),

        # app.py도 내부에서 Flask + ROS bridge node를 직접 생성합니다.
        # name remap을 생략해 app 내부 node 이름을 그대로 사용합니다.
        Node(
            package='cobotender',
            executable='app',
            output='screen',
        ),
    ])
