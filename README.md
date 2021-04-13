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
### 1. ポート権限を得る(*は番号)
```
sudo chmod a+rw /dev/ttyUSB*
```
<br>

### 2. Dynamixel(モータ)が検知できるか確認(*は番号)
```
rosrun dynamixel_workbench_controllers find_dynamixel /dev/ttyUSB*
```
このときに，以下のようなテキストが表示されるので，"id"を記録しておく(下図だと"1")
```
[ INFO] [1544589715.841211668]: Succeed to init(9600)
[ INFO] [1544589715.841236741]: Wait for scanning...
[ INFO] [1544589737.539083688]: Find 0 Dynamixels
[ INFO] [1544589737.539526809]: Succeed to init(57600)
[ INFO] [1544589737.539570059]: Wait for scanning...
[ INFO] [1544589755.441019922]: Find 2 Dynamixels
[ INFO] [1544589755.441086482]: id : 1, model name : MX-28-2
[ INFO] [1544589755.441504892]: Succeed to init(115200)
[ INFO] [1544589755.441548969]: Wait for scanning...
[ INFO] [1544589773.031677244]: Find 0 Dynamixels
[ INFO] [1544589773.032153380]: Succeed to init(1000000)
[ INFO] [1544589773.032178580]: Wait for scanning...
[ INFO] [1544589790.291943770]: Find 0 Dynamixels
[ INFO] [1544589790.292404604]: Succeed to init(2000000)
[ INFO] [1544589790.292418207]: Wait for scanning...
[ INFO] [1544589807.530702991]: Find 0 Dynamixels
[ INFO] [1544589807.531286252]: Succeed to init(3000000)
[ INFO] [1544589807.531331656]: Wait for scanning...
[ INFO] [1544589824.762803705]: Find 0 Dynamixels
[ INFO] [1544589824.763461821]: Succeed to init(4000000)
[ INFO] [1544589824.763506935]: Wait for scanning...
[ INFO] [1544589841.990120553]: Find 0 Dynamixels
```
<br>

### 3. コントローラを起動
```
roslaunch dynamixel_workbench_contrtollers roomba_joint.launch
```
エラーメッセージが出たら
```
dynamixel-workbench/dynamixel_workbench_controllers/config/roomba_joint.yaml
```
の"id"が一致していない可能性があるがあるため，先程記録した"id"を一致させる
もしくは，
```
dynamixel-workbench/dynamixel_workbench_controllers/launch/roomba_joint.launch
```
の"usb_port"が一致していない可能性があるので，異なっていたら一致させる
<br>
<br>

### 4. 回転させる

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