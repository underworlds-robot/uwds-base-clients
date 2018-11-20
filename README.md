# uwds_base_clients
This ROS package contains a basic set of [Underworlds](https://github.com/underworlds-robot/uwds) clients :

 * **ArObjectProvider** provide objects based on ar_track_alvar markers
 * **SimpleObjectProvider** provide an object based on a jsk_recognition bbox
 * **SceneViewer** read worlds an send to rviz the corresponding markers
 * **EnvProvider** load an Assimp compatible file into a world
 * **WorldMerger** merge the input worlds, allow to gather perception data
