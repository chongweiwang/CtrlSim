

## 介绍
基于qt界面的一个可以用c或c++离散化仿真简单控制对象的二次开发的框架。

电机控制qq群：528884293
个人微信公众号:王崇卫

####  软件要求
**qt版本：QT5.14.2**

百度云盘：链接：https://pan.baidu.com/s/17z17-v6hra4dGM1ZjEVcTw 
提取码：g7p6 

#### example:
1. DR_CAN老师讲课的用simulink仿真的例子。 
    - LQR
    - 轨迹跟随
    - 隆伯格观测器
    - 基础反馈线性化
    - 反步法
    - 自适应控制
    - 滑模控制
    - 高频-高增益鲁棒控制
    - 卡尔曼_递归算法

2. 电机控制仿真
    - 直流电机控制
    - PMSM_FOC id=0, 速度电流环
    - sensorless 非线性磁链观测器(本杰明电调)
    - sensorless 静态补偿电压模型(Lennart Harnefors)(陈嘉豪博士b站视频)
    - sensorless 滑模观测器(TI SMO)
    - PMSM_FOC id=0, 一阶LADR控制器

#### model:
1. 电机
    - 直流有刷电机
    - PMSM
