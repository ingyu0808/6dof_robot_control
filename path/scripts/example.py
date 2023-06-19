import moteus
import math

def motor(vel,accel):

    drive = moteus.Controller(id=1)
    
    motor = drive.make_position(
        position=math.nan,
        velocity=vel,
        kp_scale=1,
        kd_scale=1,
        maximum_torque=1,
        stop_position=math.nan,
        watchdog_timeout=math.nan,
        accel_limit=accel,
        query=True)
    

if __name__ == '__main__':
    motor(2,2)
