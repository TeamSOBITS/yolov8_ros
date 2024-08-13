import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'weights',
            default_value=os.path.join(
                os.getenv('HOME'),
                # 'colcon_ws/src/yolov8_ros/scripts/weights/azure_rcjp2022_500_v8n/best.pt'
                'colcon_ws/src/yolov8_ros/weights/opl_objects.pt'
            ),
            description='Path to the YOLOv8 weights file'
        ),
        DeclareLaunchArgument(
            'conf',
            default_value='0.3',
            description='Confidence threshold'
        ),
        DeclareLaunchArgument(
            'inference_size_h',
            default_value='640',
            description='Inference image height'
        ),
        DeclareLaunchArgument(
            'inference_size_w',
            default_value='480',
            description='Inference image width'
        ),
        DeclareLaunchArgument(
            'view_image',
            default_value='true',
            description='Visualize using OpenCV window'
        ),
        DeclareLaunchArgument(
            'save_image',
            default_value='false',
            description='Save result images'
        ),
        DeclareLaunchArgument(
            'pub_rate',
            default_value='5',
            description='Publish rate'
        ),
        DeclareLaunchArgument(
            'image_topic_name',
            # default_value='/rgb/image_raw',
            # default_value='/camera/camera/color/image_raw',
            default_value='/camera/camera/color/image_rect_raw',
            description='Image topic name'
        ),

        # YOLOv8 Node
        Node(
            package='yolov8_ros',
            executable='detect_ros.py',
            name='yolov8',
            output='screen',
            parameters=[{
                'weights': LaunchConfiguration('weights'),
                'conf': LaunchConfiguration('conf'),
                'inference_size_h': LaunchConfiguration('inference_size_h'),
                'inference_size_w': LaunchConfiguration('inference_size_w'),
                'view_image': LaunchConfiguration('view_image'),
                'save_image': LaunchConfiguration('save_image'),
                'pub_rate': LaunchConfiguration('pub_rate'),
                'initial_predict': True,
                'image_topic_name': LaunchConfiguration('image_topic_name')
            }]
        ),

        # Include the bbox_to_tf launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    os.getenv('HOME'),
                    'colcon_ws/src/bbox_to_tf/launch/bbox_to_tf.launch.py'
                )
            ]),
            launch_arguments={
                'node_name': 'yolov8_bbox_to_tf',
                'base_frame_name': 'base_footprint',
                'bbox_topic_name': '/yolov8/objects_rect',
                'cloud_topic_name': '/camera/camera/depth/color/points',
                'img_topic_name': LaunchConfiguration('image_topic_name'),
                'execute_default': "true",
                'cluster_tolerance': '0.008',
                'min_clusterSize': '10',
                'max_lusterSize': '2000000',
                'noise_point_cloud_range': '0.03',
                # 'rviz': 'True'
            }.items()
        )
    ])
