import time, sys, signal, cv2
from drone_control_nogps import Drone

# def signal_handler(signal_num, frame):
#     print("handleing")
#     print(signal_num)
#     raise KeyboardInterrupt("system quit")

# signal.signal(signal.SIGQUIT, handler = signal_handler) # ctrl + c
# signal.signal(signal.SIGINT, handler = signal_handler)  # ctrl + \

try:
    
    drone = Drone(serial_address='/dev/ttyTHS0', baud=57600)

    print("ready ")
    while(True):
        print(drone.get_altitude())
        time.sleep(1)
    """drone.arm_nogps()
    drone.takeoff_nogps(0.8)
    
    
    while(True):
        if drone.vehicle.mode == 'LAND':
            break
        
        drone.send_attitude_target(roll_angle=0.0, pitch_angle=0.0, yaw_angle = 0, use_yaw_rate = False)
        time.sleep(0.01)"""
        

except KeyboardInterrupt:
    print("[main]quit by user")
except Exception as e:
    print("[main]other exception:", e)
finally:
    print("[main]landing...")
    # drone.land()
    print("[main]landed")
    drone.vehicle.close()
    print("[main]vehicle close")
    print("[main]final")
    
