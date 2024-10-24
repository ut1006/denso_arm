# m1_research_gibberellin

アームの電源を入れる
auto modeになっていることを確認

roscore
roslaunch denso_robot_bringup vs060_bringup.launch sim:=false ip_address:=192.168.11.243
python3 MovePositionWithMoveit.py