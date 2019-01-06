# 用Python实现大疆无人机航点规划任务

[[ROS](https://wiki.ros.org/)|[大疆](https://www.dji.com/cn)|[M600 Pro](https://www.dji.com/cn/matrice600-pro?site=brandsite&from=nav)|[开发者](https://developer.dji.com/cn/onboard-sdk/)|**Python**|[SDK](https://developer.dji.com/onboard-sdk/documentation/sample-doc/sample-setup.html#ros-onboard-computer)]
----------
## 一、硬、软件环境

- 无人机：大疆M600 Pro无人机
- 飞控单元：A3 Pro
- 飞控接口：大疆[Onboard-SDK-ROS](https://github.com/dji-sdk/Onboard-SDK-ROS)
- 开发平台：树莓派ubuntu、ubuntu电脑主机
- 代码实现语言：Python

事实上，以下机型都支持

| Aircraft/FC       | Firmware Package Version | Flight Controller Version | OSDK Branch                | Notes                                                                 |
|-------------------|--------------------------|---------------------------|----------------------------|-----------------------------------------------------------------------|
| **M210/M210 RTK** | **1.1.0913**             | **3.3.10.4**              | **OSDK-ROS 3.7**           |                                                                       |
|                   |                          |                           |                            |                                                                       |
| **M600/M600 Pro** | **1.0.1.66**             | **3.2.41.13**             | **OSDK-ROS 3.7**           |                                                                       |
|                   |                          |                           |                            |                                                                       |
| **A3/A3 Pro**     | **1.7.6.0**              | **3.3.8.39**              | **OSDK-ROS 3.7**           |                                                                       |
|                   |                          |                           |                            |                                                                       |
| **N3**            | **1.7.6.0**              | **3.3.8.39**              | **OSDK-ROS 3.7**           |                                                                       |
|                   |                          |                           |                            |                                                                       |
| **M100**          | 1.3.1.82                 | **3.1.10.0**              | **OSDK-ROS 3.7**           |                                                                       |

## 二、航点规划任务的流程图

![流程图](images/missions_sample_flowchart.png)

## 三、流程的基本步骤

 1. 按照大疆官网**安装Onboard-SDK-ROS** ，[链接](https://developer.dji.com/onboard-sdk/documentation/sample-doc/sample-setup.html#ros-onboard-computer)；
 2. **导入相关的包**，其中`drone_proxy`这个包定义了一系列无人机对象和方法，需要付费才能获取哦 ；
 
```python
#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from drone_proxy import *
```

 3.  **激活无人机并获取SDK控制权限** ；

```python
drone = Drone()
ret = drone.activate()
if ret.result:
    print "无人机激活成功!"
ret = drone.obtain_authority()
if ret.result:
    print "SDK获取无人机控制权限成功!"
else:
    if ret.ack_data == 3 and ret.cmd_set == 1 and ret.cmd_id == 0:
        print "正在获取控制权限，再发一次请求..."
        drone.obtain_authority()
    else:
        print "获取控制权限失败!"
```

 4.  **ROS节点初始化** 并订阅当前位置；

```python
rospy.init_node('demo_waypoint', anonymous=True) 
ori_pos = rospy.client.wait_for_message("/dji_sdk/gps_position", NavSatFix, 2) 
```

 5. 定义航点属性函数；

```python
def waypoint_property(latitude, longitude, altitude, heading=0, hovertime=5):
    """航点属性赋值函数"""
    wp = MissionWaypoint()
    wp.latitude = latitude
    wp.longitude = longitude
    wp.altitude = altitude
    wp.damping_distance = 0  # Bend length (effective coordinated turn mode only)
    wp.target_yaw = heading  # 目标偏航角
    wp.target_gimbal_pitch = 0  #TODO
    wp.turn_mode = 0  # 转弯模式 0顺时针方向 1逆时针方向
    # 设置悬停
    wp.has_action = 1  # 有没有动作 0无 1有
    wp.action_time_limit = 5000
    wp.waypoint_action.action_repeat = 1
    wp.waypoint_action.command_list = "\0"*16
    wp.waypoint_action.command_parameter[0] = hovertime * 1000
    return wp
```

 6. 定义航点任务；

```python
def waypoint_mission(p):
    # 航点任务基本描述
    bp = MissionWaypointTask()
    bp.velocity_range = 10
    bp.idle_velocity = 5
    bp.action_on_finish = 0  # 结束后的动作 0无动作 1返回原点 2自动着陆 3返回某点 4无尽模式，不退出
    bp.mission_exec_times = 1  # 仅执行一次
    bp.yaw_mode = 0  # 航向（机头方向） 0自动模式（指向下一个航点） 1锁定初始值 2由遥控器控制 3采用航点的偏航角
    bp.trace_mode = 0  # 转弯模式为定点转弯
    bp.action_on_rc_lost = 1  # RC失联后执行失联保护 #TODO
    bp.gimbal_pitch_mode = 0  
    # 添加航点
    wps = []
    for i in range(len(p)):
        heading_angle = 0  # 机头朝向
        wp = waypoint_property(p[i][0], p[i][1], p[i][2], heading=heading_angle)
        wps.append(wp)
    bp.mission_waypoint = wps
    return bp
```

 7. 装订航线并启动任务；

```python
def load_and_start(mission):
    # 装订航点
    ret = drone.wpMission.upload(mission)
    if ret.result:
        print "航点装订成功!"
        ret = drone.wpMission.start()
        if ret.result:
            print "航点任务启动成功!"
        # 开始ROS调度
        try:
            rospy.spin()
        except KeyboardInterrupt, e:
            pass
```

 8. 定义主函数，在主函数中定义航点，并调用函数。

```python
def main():
    if isinstance(ori_pos, NavSatFix):
        origin_p = (ori_pos.latitude, ori_pos.longitude, ori_pos.altitude)
        points = []
        next_p = (origin_p[0], origin_p[1], origin_p[2]+12)
        points.append(next_p)
        next_p = (origin_p[0]+0.00030, origin_p[1], origin_p[2]+12)
        points.append(next_p)
        next_p = (origin_p[0]+0.00030, origin_p[1]+0.00020, origin_p[2]+12)
        points.append(next_p)
        next_p = (origin_p[0], origin_p[1]+0.00020, origin_p[2]+12)
        points.append(next_p)
        print "总共有{}个点，各点的坐标为\n{}".format(len(points), points)

        task = waypoint_mission(points)
        load_and_start(task)


if __name__ == "__main__":
    main()

```

**提示：**
- 1、以上航点实现的是长方形飞行，如果想实现来回往返的多航点飞行，就需要规划好航点，传给 `waypoint_mission()` 函数；
- 2、由于室内GPS信号弱，无人机并不会启动飞行。

## 四、`drone_proxy`包的获取方式

扫二维码，支付宝付款成功后，加微信好友，备注**drone_proxy**

![支付宝付款码](images/pay_qrcode.jpg)

---

![付款成功后，备注drone_proxy加好友](images/add_friends.jpg)
