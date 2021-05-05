
# 打开can口(如果有需要，运行代码文件之前需要打开can口)

sudo ip link set can0 type can bitrate 1000000  
sudo ip link set up can0  

# 关闭roscore（因为通过ssh进行调试，会存在roscore难以关闭的情况）

killall -9 roscore  
killall -9 rosmaster  
roscore  

# 请不要在运行代码时按键CREL + Z；可以按ctrl + c来停止代码

# 简单的demo
终端运行
mkdir ~/catkin_ws  
mkdir src  
cd src 
把代码放在src目录下面
然后终端运行：
cd ~/catkin_ws  
catkin_make  
source devel/setup.bash  
rosrun cantest canTest1  
---------------------
## canTest4

    整理代码文件，增加模式功能，可以在运行时选择不同的模式

    mode；初始mode为0
    default:更改mode参数为0
    0:只接受电机传来的数据信息，四个电机的发送电流全为0
    1:速度环PI参数调节模式，初始目标速度为0，通过topic{targetVelocity,PI}来进行调参（单个电机，ID为0）
    2:位置环P参数调节模式，初始位置为0，通过topic{targetPosition,PI}该模式下使用上一步测试得到的速度环PI参数/单个电机，ID为0）
    3:速度环控制模式，初始目标速度为0；通过topic{targetVelocity}设置目标速度（两个个电机，ID为0）
    4:位置环控制模式，初始目标位置为0；通过topic{targetPosition}设置目标位置（单个电机，ID为0）
    5:位置环控制模式，初始获得电机当前位置设定为目标位置；通过topic{targetPositionALL}设定目标位置（四个电机，ID为0,1，2，3）
    6:扫描行程模式，得到四个电机的行程，建立相对坐标系；然后执行mode0
    7:暂定

    完成扫描得到电机行程极限数据

## canTest3

    读取四个电机当前位置信息，然后把位置信息设定位目标位置，进行位置环控制

## canTest2

    单个电机速度环 （PI控制）合适的PI参数
    然后在速度环外放置位置环进行位置控制
    在matlab端发送控制信息
    实验结果：PPI 15,13,6

## canTest1

    读取电机数据（机械角度，速度，温度，控制电流）
