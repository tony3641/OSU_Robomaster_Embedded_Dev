May 18, 2019
完成工程车机械臂控制程序，待调用
完成英雄车程序气动炮GPIO，待测试
完成防止自瞄开启时抖动，通过加一个delay
完成软件复位
完成开机时云台保持在原位，通过在while(1)loop前的   GIMBAL_Feedback_Update(&gimbal_control);             //云台数据反馈   前增加delay
GroupDelayTable.xlsx没用了，想删就删

May 14, 2019
完成所有GPIO及PWM配置和封装

May 11, 2019
完成自瞄预测，效果优化基本完成
完成GPIO初步调用
目标：	自瞄开启时会抖动
	Erase Full Chip时发现小bug，待修复

May 9, 2019
由于陀螺仪范围是（-π，π），将自瞄开启时的陀螺仪读值设为0以免启动时因为超出范围而晃飞
目标：	Native Prediction
	Erase Full Chip时发现小bug，待修复
	Pitch轴有点晃

May 7, 2019
更新四阶Kalman Filter，完成扭腰模式
目标：	Native Prediction
	发送给C620的电调数据不一定是真实的电流，底盘功率限制

May 1, 2019
Kalman Filter测试完成，任意角度开启自瞄完成
延迟太大了，扭腰模式再议
目标：	与速度结合完成预测
	完成底盘功率限制
	修改跟随模式为直接操控底盘旋转，云台锁定不动

April 24, 2019
优化初步完成，选用Butterworth IIR(50ms Group Delay)，CV发送延迟(约190ms)，Kalman学习中
目标：	定义当前relative_angle为reference，以便在任意角度开启自瞄
	尽可能减小gyro的速度控制影响以便扭腰模式不受干扰
	对底盘进行system identification，尝试使用lead/lag compensator instead of PID，尝试减小急停时的功率
	或者缓速停止而非直接到0

April 23, 2019
上机测试初步成功，需要平滑CV数据，提升CV发送帧率，降低CV发送延迟(约200ms)

April 22, 2019
分线程（user_task.c）用于filter计算，CV传输来的数据(有延迟时间t)与延迟时间t前的编码器值计算后抵消得到一个定值，用于旋转云台的目标位置

April 14, 2019
更改为位置PID锁定（测试）
加入滤波器，启用ChebyshevII LPF平滑tx2传输数据，创建filter.c/.h，整理代码

April 11, 2019
通过CAN发送云台陀螺仪数据

April 9, 2019
加入电子限制电机限位

April 3, 2019
修复滤波器，加入卡尔曼滤波，有一定预测效果
DBUS连接松动导致底盘不定时断线

March 26, 2019
摩擦轮，供弹轮转速调整，加入自适应PID（仅Kp）

March 17, 2019
Erase Full Chip可重新校准陀螺仪，完成键盘鼠标映射，一键切换扭腰模式

March 15, 2019
无预测PID调试基本完成

March 13, 2019
摄像机放云台上追随识别框成功

Feb 26, 2019
英雄底盘组装完毕

Feb 24, 2019
初步判断是CAN通信优先级导致的丢包和延时，更改CAN通信数据优先级，将TX2的CAN_ID优先级调至最高（0x208->0x111），发现并无太大关系（0x111->0x300依然测试成功）。
单独定义并测试自瞄模块程序（get_aim_data）控制云台，测试成功
说明get_tx2_measure程序switch失效，暂时弃用
测试TX2发送数据包至步兵车来控制云台，测试成功
测试TX2视觉程序，通过摄像头读取其他步兵车的数字装甲板后计算，发送坐标数据包至步兵车控制云台，实现步兵车云台跟随其他步兵车的数字装甲板，测试成功

Feb 23, 2019
加入由CAN通信接口控制云台的代码，控制云台测试成功，然而TX2自瞄数据包大量丢包延时，效果很差
调试由CAN通信接口发至TX2的编码器读数，测试成功

Feb 22, 2019
加入控制云台的遥控器/鼠标代码控制云台，测试成功
加入get_tx2_measure程序，由Data[0]作为标识符判断收到的数据包类型（调整pitch/yaw PID数值数据包，自瞄数据包），未测试

Feb 15, 2019
第一次成功烧录步兵车程序，然而遥控器只能控制底盘和射击

Jan 30, 2019
熟悉并理解代码
