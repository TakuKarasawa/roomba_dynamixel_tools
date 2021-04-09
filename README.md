# roomba_dynamixel_tools

## インストールとビルド
事前にDynamixelSdkとDynamixel,Dynamixel-workbench,dynamixel-workbench-msgsをインストールしておく必要がある

* DynamixelSdk 
```
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```
* Dynamixel-workbench
```
git clone https://github.com/TakuKarasawa/dynamixel-workbench.git
```
* Dynamixel-workbench-msgs
```
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
```

## Dynamixel(モータ)の起動方法
* ポート権限を得る(*は番号)
```
sudo chmod a+rw /dev/ttyUSB*
```
* Dynamixel(モータ)が検知できるか確認(*は番号)
```
rosrun dynamixel_workbench_controllers find_dynamixel /dev/ttyUSB*
```
* コントローラを起動
```
roslaunch dynamixel_workbench_contrtollers roomba_joint.launch
```
エラーメッセージが出たら
```
dynamixel-workbench/dynamixel_workbench_controllers/config/roomba_joint.yaml
```
の"usb_port"が一致していない可能性がある

* 回転

```
roslaunch roomba_dynamixel_controller teleop_roomba_dynamixel.launch
```
作動しない場合は，
```
dynamixel_workbench/dynamixel_workbench_controller/config/roomba_joint.yaml
```
の"dynamixel"と
```
roomba_dynamixel_tools/roomba_dynamixel_controller/src/roomba_dynamixel.cpp
```
の"jt.joint_name[0]= が一致していない可能性があるので一致させる