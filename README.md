GNSS/Visual/INS 系统硬件驱动代码

软件环境：</p>
ROS1-Noetic</p>
Ubuntu20.04</p>

硬件环境：</p>
IMU: STIM300</p>
GNSS：u-blox NEO-M9N</p>
Camera：DECXIN-1M-SM-2296V1</p>

说明：</p>
工作空间中包含三个功能包，分别用来驱动STIM300 IMU、u-blox NEO-M9N、DECXIN-1M-SM-2296V1三个模块。</p>
使用时catkin_make整个工作空间，之后rosrun对应功能包的节点即可（注意程序中设备号可能和你的不相同，需要修改，可以使用cutecom等工具进行串口收发测试，确定串口号）</p>
