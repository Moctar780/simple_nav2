# simple_nav2
# launch test
ros2 launch move_base task_solution_launch.py filename:=move_base/routes/world use_rviz:=True use_sim_time:=True
![alt text](<Screenshot from 2025-07-27 10-20-34.png>)
![alt text](<Screenshot from 2025-07-27 10-20-45.png>)
# launch record new trajectory
ros2 launch move_base task_solution_launch.py filename:=move_base/routes/new_record option:=record use_move_server:=False
# display tarjet
ros2 run move_base pose_manage.py --ros-args -p option:=display -p filename:=move_base/routes/world
