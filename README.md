Please refer to https://github.com/DENSORobot/denso_robot_ros


### 実機動かす手順
①(必要であれば) catkin build のインストール
```
$ sudo apt install python3-catkin-tools
```
② moveitのインストール
```
$ sudo apt install ros-noetic-moveit
```
③ ros control周りのインストール
```
$ sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers
```
④ denso_robot パッケージのインストール
```
$ mkdir -p catkin/src
$ git clone https://github.com/yamachaso/denso_robot_ros_for_noetic.git
$ catkin build
$ source ~/catkin/devel/setup.bash
```
⑤  /denso_robot_descriptions/vs060_description/vs060.launch.xml を書き換える。
```
    <param name="robot_name" value="VS060A3-AV6-W4N-ANN"/>
```
にする

⑥ ネットワーク設定
イーサネットで接続する。Wi-Fi(Baffalo-G-D290)経由で接続すると指令値生成遅延が発生。

⑦ roslaunch起動
```
$ roslaunch denso_robot_bringup vs060_bringup.launch sim:=false ip_address:=192.168.11.100
```
bringup.shに記述されている。


