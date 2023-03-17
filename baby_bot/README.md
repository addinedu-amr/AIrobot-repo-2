# dependency manual
``` bash
pip install opencv-python
export PYTHONPATH=/$your_path$/mini_bot/src/pinklab_minibot_robot/minibot_indoor/minibot_indoor:$PYTHONPATH
```


# running simulation
``` bash
ros2 launch minibot_bringup bringup_robot_gazebo.launch.py world_name:=my.world
ros2 launch minibot_navigation2 bringup_launch.py use_sim_time:=true map:=`ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/maps/baby_map.yaml
rviz2 -d `ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/rviz/nav2_view.rviz
ros2 run minibot_indoor order
ros2 run minibot_indoor path_planning
```
