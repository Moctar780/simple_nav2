# simple_nav2
# launch test
ros2 launch move_base task_solution_launch.py filename:=move_base/routes/world option:=play use_move_server:=True
# launch record new trajectory
ros2 launch move_base task_solution_launch.py filename:=move_base/routes/new_record option:=record use_move_server:=False
# display tarjet
ros2 run move_base pose_manage.py --ros-args -p option:=display -p filename:=move_base/routes/world
