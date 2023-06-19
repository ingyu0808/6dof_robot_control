#!/usr/bin/env python3

import rospy
import asyncio
import time
import moteus
import math
from queue import Queue
import threading
from moveit_msgs.msg import DisplayTrajectory

control_queue = Queue()



#----------motor init setting ---------------#

async def motor1(pos,vel,accel):   #motor1#
    
    drive = moteus.Controller(id=2)
    
    motor1 =await drive.set_position(
        position=pos,
        velocity=vel,
        kp_scale=1,
        kd_scale=1,
        maximum_torque=1,
        stop_position=math.nan,
        watchdog_timeout=math.nan,
        accel_limit=accel,
        query=True)
    
    
2
async def motor2(pos,vel,accel):  #motor2#
    
    drive = moteus.Controller(id=1)
    
    motor2 =await drive.set_position(
        position=pos,
        velocity=vel,
        kp_scale=1,
        kd_scale=1,
        maximum_torque=1,
        stop_position=math.nan,
        watchdog_timeout=math.nan,
        accel_limit=accel,
        query=True)
    


async def motor3(pos,vel,accel):  #motor3#
    
    drive = moteus.Controller(id=3)
    
    motor3 =await drive.set_position(
        position=pos,
        velocity=vel,
        kp_scale=1,
        kd_scale=1,
        maximum_torque=1,
        stop_position=math.nan,
        watchdog_timeout=math.nan,
        accel_limit=accel,
        query=True)
    

async def motor4(pos,vel,accel):  #motor4#
    
    drive = moteus.Controller(id=4)
    
    motor4 =await drive.set_position(
        position=pos,
        velocity=vel,
        kp_scale=1,
        kd_scale=1,
        maximum_torque=1,
        stop_position=math.nan,
        watchdog_timeout=math.nan,
        accel_limit=accel,
        query=True)
    


async def motor5(pos,vel,accel):  #motor5#
    
    drive = moteus.Controller(id=5)
    
    motor5 =await drive.set_position(
        position=pos,
        velocity=vel,
        kp_scale=1,
        kd_scale=1,
        maximum_torque=1,
        stop_position=math.nan,
        watchdog_timeout=math.nan,
        accel_limit=accel,
        query=True)
    


async def motor6(pos,vel,accel):  #motor6#
    
    drive = moteus.Controller(id=6)
    
    motor6 =await drive.set_position(
        position=pos,
        velocity=vel,
        kp_scale=1,
        kd_scale=1,
        maximum_torque=1,
        stop_position=math.nan,
        watchdog_timeout=math.nan,
        accel_limit=accel,
        query=True)
    



def motor_sch_thread(loop):
    asyncio.set_event_loop(loop)
    global control_queue
    execute_time = 0
    while True:
        if control_queue.empty():
            continue
        current_input = control_queue.get()
        #print("queue", current_input, rospy.Time.now())
        #loop.run_until_complete(motor1(current_input[0][0],current_input[0][1], abs(current_input[0][2])))
        #loop.run_until_complete(motor2(current_input[0][3],current_input[0][4], abs(current_input[0][5])))
        # loop.run_until_complete(motor3(current_input[0][6],current_input[0][7], abs(current_input[0][8])))
        # loop.run_until_complete(motor4(current_input[0][9],current_input[0][10], abs(current_input[0][11])))
        # loop.run_until_complete(motor5(current_input[0][12],current_input[0][13], abs(current_input[0][14])))
        # loop.run_until_complete(motor6(current_input[0][15],current_input[0][16], abs(current_input[0][17])))        
        time.sleep(abs(current_input[0][6]))




def callback(data : DisplayTrajectory):
    global control_queue
    previous_time = 0

    for i in range(len(data.trajectory[0].joint_trajectory.points)):
        print("--------------------point",i,"-------------------------")
        current_time = round(((data.trajectory[0].joint_trajectory.points[i].time_from_start.secs)+(data.trajectory[0].joint_trajectory.points[i].time_from_start.nsecs)*1e-9),4)
        loop_time = (current_time - previous_time)
        previous_time = current_time
        print('looptime : ',round((loop_time),4))

        pos1 = round((data.trajectory[0].joint_trajectory.points[i].positions[0]),4)*(180/math.pi)*(100/360)
        vel1 = round((data.trajectory[0].joint_trajectory.points[i].velocities[0]),4)*(180/math.pi)*(100/360)
        acc1 = round((data.trajectory[0].joint_trajectory.points[i].accelerations[0]),4)*(180/math.pi)*(100/360)

        pos2 = round((data.trajectory[0].joint_trajectory.points[i].positions[1]),4)*(180/math.pi)*(100/360)
        vel2 = round((data.trajectory[0].joint_trajectory.points[i].velocities[1]),4)*(180/math.pi)*(100/360)
        acc2 = round((data.trajectory[0].joint_trajectory.points[i].accelerations[1]),4)*(180/math.pi)*(100/360)
        
        print(vel1)

        # pos3 = round((data.trajectory[0].joint_trajectory.points[i].positions[2]),4)*(180/math.pi)*(50/360)
        # vel3 = round((data.trajectory[0].joint_trajectory.points[i].velocities[2]),4)*(180/math.pi)*(50/360)
        # acc3 = round((data.trajectory[0].joint_trajectory.points[i].accelerations[2]),4)*(180/math.pi)*(50/360)

        # pos4 = round((data.trajectory[0].joint_trajectory.points[i].positions[3]),4)*(180/math.pi)*(50/360)
        # vel4 = round((data.trajectory[0].joint_trajectory.points[i].velocities[3]),4)*(180/math.pi)*(50/360)
        # acc4 = round((data.trajectory[0].joint_trajectory.points[i].accelerations[3]),4)*(180/math.pi)*(50/360)

        # pos5 = round((data.trajectory[0].joint_trajectory.points[i].positions[4]),4)*(180/math.pi)*(50/360)
        # vel5 = round((data.trajectory[0].joint_trajectory.points[i].velocities[4]),4)*(180/math.pi)*(50/360)
        # acc5 = round((data.trajectory[0].joint_trajectory.points[i].accelerations[4]),4)*(180/math.pi)*(50/360)

        # pos6 = round((data.trajectory[0].joint_trajectory.points[i].positions[5]),4)*(180/math.pi)*(50/360)
        # vel6 = round((data.trajectory[0].joint_trajectory.points[i].velocities[5]),4)*(180/math.pi)*(50/360)
        # acc6 = round((data.trajectory[0].joint_trajectory.points[i].accelerations[5]),4)*(180/math.pi)*(50/360)


        # for j in range(1,7):
        # pos = round((data.trajectory[0].joint_trajectory.points[i].positions[1]),4)*180/math.pi
        # vel = round((data.trajectory[0].joint_trajectory.points[i].velocities[1]),4)
        # accel = round((data.trajectory[0].joint_trajectory.points[i].accelerations[1]),4)
        #     print("joint",j)
        # print("    posiiton:", pos) 
        # print("    velocity:", vel)
        # print("    acceleration:", accel)
        # print(" ")

        control_queue.put([
            [pos1, vel1, acc1, pos2, vel2, acc2,loop_time]
            # [pos1, vel1, acc1, pos2, vel2, acc2,pos3, vel3, acc3,pos4, vel4, acc4,pos5, vel5, acc5, pos6, vel6, acc6, loop_time]
        ])
        
        


def listener():
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=motor_sch_thread, args=(loop,))
    
    rospy.init_node('trajectory_subscriber1')
    rospy.Subscriber("move_group/display_planned_path", DisplayTrajectory, callback)

    thread.start()
    rospy.spin()
    thread.join()

if __name__ == '__main__':
    listener()