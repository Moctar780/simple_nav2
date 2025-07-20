"""Command line helper"""
1-> launch record poses
$ ros2 launch move_base task_solution_launch.py use_move_server:=False use_pose_manage:=True option:=record filename:=route1_track_1 convert_topic:=odom_filtered use_estimator:=True use_rviz:=True

commentaire: use_move_server( il est deactiver juste un record)
	     convert_topic (si tu utilise distance Estimator alors 'odom_filtered', odom simple 'odom', ou bien odom_typic)
             option: (si move_server est deactiver, option:play aussi est deactiver)
             filename: un fichier poses pour lire ou ecrire des poses

2-> launch play poses
$ ros2 launch move_base task_solution_launch.py use_move_server:=True use_pose_manage:=True option:=play filename:=routes/route1_teleop.txt convert_topic:=odom_filtered use_estimator:=True use_rviz:=True
