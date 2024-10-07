from queue import Empty
from process import AvoidProcessing
import time, sys, signal, cv2
import traceback

SIMULATE = False

if not SIMULATE:
    
    gstream_pipeline = (
        "nvarguscamerasrc sensor-id=0 sensor_mode=4 ! "
        "video/x-raw(memory:NVMM), "
        "width=(int){capture_width:d}, height=(int){capture_height:d}, "
        "format=(string)NV12, framerate=(fraction){framerate:d}/1 ! "
        "nvvidconv flip-method={flip_method:d} ! "
        "video/x-raw, width=(int){display_width:d}, height=(int){display_height:d}, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink".format(
            capture_width=1280,
            capture_height=720,
            framerate=30,
            flip_method=2,
            display_width=400,
            display_height=225,
        )
    )

    print(gstream_pipeline)

def signal_handler(signal_num, frame):
    print("handleing")
    print(signal_num)
    raise KeyboardInterrupt("system quit")

signal.signal(signal.SIGQUIT, handler = signal_handler) # ctrl + c
signal.signal(signal.SIGINT, handler = signal_handler)  # ctrl + \

try:
    
    if SIMULATE:
        avoid_processing = AvoidProcessing(simulate=True)
    else:
        avoid_processing = AvoidProcessing(gstream_def=gstream_pipeline, simulate=False)
    
    drone = avoid_processing.drone
    
    avoid_processing.record_start=True
    
    print(avoid_processing.process_running.get())
    drone.arm_nogps()
    drone.takeoff_nogps(0.8)
    
    
    while(True):
        if drone.vehicle.mode == 'LAND':
            break
        
        control = avoid_processing.get_control()
       # print('[main]control:', control)
        # time.sleep(0.1)
        roll, pitch, yaw, thrust = control
        drone.send_attitude_target(roll_angle=roll, pitch_angle=pitch, yaw_rate = yaw, thrust = thrust)
        time.sleep(0.01)
        drone.send_attitude_target(roll_angle=0.0, pitch_angle=0.0, yaw_rate = yaw, thrust = thrust)
        print("[main] roll: %1.4f, pitch: %1.4f, yaw: %1.4f"%(roll, pitch, yaw))
        # print("[main] thrust: ", thrust)
        
        # exc = avoid_processing.track.exception
        # if exc:
        #     error, tb = exc 
        #     print(error, tb)
        #     raise ValueError("error in avoiding.track")
        # try:
        #     exc = avoid_processing.exception_list.get(block=False)
        # except Empty:
        #     pass
        # else:
        #     exc_type, exc_obj, exc_trace = exc
        #     # deal with the exception
        #     traceback.print_exception(*exc)
        #     del exc
        #     break
    # drone.land()

except KeyboardInterrupt:
    print("[main]quit by user")
except Exception as e:
    print("[main]other exception:", e)
    # traceback.print_exception(*sys.exc_info())
finally:
    avoid_processing.show_save_err_plt()
    print("[main]img save finished")
    print("[main]landing...")
    drone.land()
    print("[main]landed")
    # drone.vehicle.close()
    # print("[main]vehicle close")
    avoid_processing.release()
    print("[main]final")
    
