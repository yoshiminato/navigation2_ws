```mermaid
flowchart LR

/nav2_parallel_env[ /nav2_parallel_env ]:::main
/bridge_ros_gz_clock[ /bridge_ros_gz_clock ]:::node
/robot_1/amcl[ /robot_1/amcl ]:::node
/robot_1/bridge_ros_gz[ /robot_1/bridge_ros_gz ]:::node
/robot_1/local_costmap/local_costmap[ /robot_1/local_costmap/local_costmap ]:::node
/robot_1/robot_state_publisher[ /robot_1/robot_state_publisher ]:::node
/robot_1/controller_server[ /robot_1/controller_server ]:::node
/robot_1/global_costmap/global_costmap[ /robot_1/global_costmap/global_costmap ]:::node
/clock([ /clock<br>rosgraph_msgs/msg/Clock ]):::topic
/robot_1/amcl_pose([ /robot_1/amcl_pose<br>geometry_msgs/msg/PoseWithCovarianceStamped ]):::topic
/robot_1/contact([ /robot_1/contact<br>ros_gz_interfaces/msg/Contacts ]):::topic
/robot_1/imu([ /robot_1/imu<br>sensor_msgs/msg/Imu ]):::topic
/robot_1/local_costmap/costmap([ /robot_1/local_costmap/costmap<br>nav_msgs/msg/OccupancyGrid ]):::topic
/robot_1/odom([ /robot_1/odom<br>nav_msgs/msg/Odometry ]):::topic
/robot_1/tf([ /robot_1/tf<br>tf2_msgs/msg/TFMessage ]):::topic
/robot_1/tf_static([ /robot_1/tf_static<br>tf2_msgs/msg/TFMessage ]):::topic
/robot_1/transformed_global_plan([ /robot_1/transformed_global_plan<br>nav_msgs/msg/Path ]):::topic
/robot_1/cmd_vel_rl([ /robot_1/cmd_vel_rl<br>geometry_msgs/msg/Twist ]):::topic
/robot_1/initialpose([ /robot_1/initialpose<br>geometry_msgs/msg/PoseWithCovarianceStamped ]):::topic
/nav2_parallel_env/get_type_description[/ /nav2_parallel_env/get_type_description<br>type_description_interfaces/srv/GetTypeDescription \]:::bugged
/robot_1/global_costmap/clear_entirely_global_costmap[/ /robot_1/global_costmap/clear_entirely_global_costmap<br>nav2_msgs/srv/ClearEntireCostmap \]:::service
/world/follow_path/create[/ /world/follow_path/create<br>ros_gz_interfaces/srv/SpawnEntity \]:::service
/world/follow_path/remove[/ /world/follow_path/remove<br>ros_gz_interfaces/srv/DeleteEntity \]:::service
/navigate_to_pose{{ /navigate_to_pose<br>nav2_msgs/action/NavigateToPose }}:::bugged
/navigate_to_pose{{ /navigate_to_pose<br>nav2_msgs/action/NavigateToPose }}:::bugged
/navigate_to_pose{{ /navigate_to_pose<br>nav2_msgs/action/NavigateToPose }}:::bugged
/navigate_to_pose{{ /navigate_to_pose<br>nav2_msgs/action/NavigateToPose }}:::bugged
/navigate_to_pose{{ /navigate_to_pose<br>nav2_msgs/action/NavigateToPose }}:::bugged
/navigate_to_pose{{ /navigate_to_pose<br>nav2_msgs/action/NavigateToPose }}:::bugged
/clock --> /nav2_parallel_env
/robot_1/amcl_pose --> /nav2_parallel_env
/robot_1/contact --> /nav2_parallel_env
/robot_1/imu --> /nav2_parallel_env
/robot_1/local_costmap/costmap --> /nav2_parallel_env
/robot_1/odom --> /nav2_parallel_env
/robot_1/tf --> /nav2_parallel_env
/robot_1/tf_static --> /nav2_parallel_env
/robot_1/transformed_global_plan --> /nav2_parallel_env
/robot_1/initialpose --> /robot_1/amcl
/robot_1/cmd_vel_rl --> /robot_1/controller_server
/nav2_parallel_env --> /robot_1/cmd_vel_rl
/nav2_parallel_env --> /robot_1/initialpose
/bridge_ros_gz_clock --> /clock
/robot_1/amcl --> /robot_1/amcl_pose
/robot_1/amcl --> /robot_1/tf
/robot_1/bridge_ros_gz --> /robot_1/contact
/robot_1/bridge_ros_gz --> /robot_1/imu
/robot_1/bridge_ros_gz --> /robot_1/odom
/robot_1/bridge_ros_gz --> /robot_1/tf
/robot_1/local_costmap/local_costmap --> /robot_1/local_costmap/costmap
/robot_1/robot_state_publisher --> /robot_1/tf
/robot_1/robot_state_publisher --> /robot_1/tf_static
/robot_1/controller_server --> /robot_1/transformed_global_plan
/nav2_parallel_env/get_type_description o-.-o /nav2_parallel_env
/world/follow_path/create o-.-o /bridge_ros_gz_clock
/world/follow_path/remove o-.-o /bridge_ros_gz_clock
/robot_1/global_costmap/clear_entirely_global_costmap o-.-o /robot_1/global_costmap/global_costmap
/nav2_parallel_env <-.-> /robot_1/global_costmap/clear_entirely_global_costmap
/nav2_parallel_env <-.-> /world/follow_path/create
/nav2_parallel_env <-.-> /world/follow_path/remove

/nav2_parallel_env <==> /navigate_to_pose
/nav2_parallel_env <==> /navigate_to_pose
/nav2_parallel_env <==> /navigate_to_pose
/nav2_parallel_env <==> /navigate_to_pose
/nav2_parallel_env <==> /navigate_to_pose
/nav2_parallel_env <==> /navigate_to_pose
subgraph keys[<b>Keys<b/>]
subgraph nodes[<b><b/>]
topicb((No connected)):::bugged
main_node[main]:::main
end
subgraph connection[<b><b/>]
node1[node1]:::node
node2[node2]:::node
node1 o-.-o|to server| service[/Service<br>service/Type\]:::service
service <-.->|to client| node2
node1 -->|publish| topic([Topic<br>topic/Type]):::topic
topic -->|subscribe| node2
node1 o==o|to server| action{{/Action<br>action/Type/}}:::action
action <==>|to client| node2
end
end
classDef node opacity:0.9,fill:#2A0,stroke:#391,stroke-width:4px,color:#fff
classDef action opacity:0.9,fill:#66A,stroke:#225,stroke-width:2px,color:#fff
classDef service opacity:0.9,fill:#3B8062,stroke:#3B6062,stroke-width:2px,color:#fff
classDef topic opacity:0.9,fill:#852,stroke:#CCC,stroke-width:2px,color:#fff
classDef main opacity:0.9,fill:#059,stroke:#09F,stroke-width:4px,color:#fff
classDef bugged opacity:0.9,fill:#933,stroke:#800,stroke-width:2px,color:#fff
style keys opacity:0.15,fill:#FFF
style nodes opacity:0.15,fill:#FFF
style connection opacity:0.15,fill:#FFF
linkStyle 151,152,153,154,155,156,162 fill:none,stroke:green;
```
