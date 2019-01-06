#!/usr/bin/python
# -*- coding: utf-8 -*-


from drone_proxy import *

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

rospy.init_node('demo_waypoint', anonymous=True)  # 节点初始化，必须的步骤
ori_pos = rospy.client.wait_for_message("/dji_sdk/gps_position", NavSatFix, 2)  # 订阅当前位置


def waypoint_property(latitude, longitude, altitude, heading=0, velocity=0.5, itype=1, hovertime=5):
    """航点属性赋值函数"""
    wp = MissionWaypoint()
    wp.latitude = latitude
    wp.longitude = longitude
    wp.altitude = altitude
    wp.damping_distance = 0  # Bend length (effective coordinated turn mode only)
    wp.target_yaw = heading  # 目标偏航角 #TODO
    wp.target_gimbal_pitch = 0  #TODO
    wp.turn_mode = 0  # 转弯模式 0顺时针方向 1逆时针方向
    # 设置悬停
    wp.has_action = 1  # 有没有动作 0无 1有  #MissionWpAction 0start 1stop 2pause 3resume #TODO
    wp.action_time_limit = 5000
    wp.waypoint_action.action_repeat = 1
    wp.waypoint_action.command_list = "\0"*16
    wp.waypoint_action.command_parameter[0] = hovertime * 1000
    return wp


def waypoint_mission(p):
    # 航点任务基本描述
    bp = MissionWaypointTask()
    bp.velocity_range = 10
    bp.idle_velocity = 5
    bp.action_on_finish = 0  # 结束后的动作 0无动作 1返回原点 2自动着陆 3返回某点 4无尽模式，不退出
    bp.mission_exec_times = 1  # 仅执行一次
    bp.yaw_mode = 0  # 航向（机头方向） 0自动模式（指向下一个航点） 1锁定初始值 2由遥控器控制 3采用航点的偏航角 #TODO
    bp.trace_mode = 0  # 转弯模式为定点转弯
    bp.action_on_rc_lost = 1  # RC失联后执行失联保护 #TODO
    bp.gimbal_pitch_mode = 0  #
    # 添加航点
    wps = []
    for i in range(len(p)):
        heading_angle = 0  # 机头朝向，360为离开该航点时机头指向下一航点
        wp = waypoint_property(p[i][0], p[i][1], p[i][2], heading=heading_angle)
        wps.append(wp)
    bp.mission_waypoint = wps
    return bp


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

