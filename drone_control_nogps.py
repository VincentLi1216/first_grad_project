"""
© Copyright 2015-2016, 3D Robotics.
guided_set_speed_yaw.py: (Copter Only)

This example shows how to move/direct Copter and send commands in GUIDED mode using DroneKit Python.

This file has been modified by Arthur Findelair. Find original script in the DroneKit example folder:
https://github.com/dronekit/dronekit-python/blob/master/examples/guided_set_speed_yaw/guided_set_speed_yaw.py

Example documentation: http://python.dronekit.io/examples/guided-set-speed-yaw-demo.html

"""


import time, math
import numpy as np

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
# from my_vehicle import MyVehicle #Our custom vehicle class


class Drone:

    def __init__(self, serial_address: str, baud: int):
        # self.vehicle = connect(serial_address, wait_ready=True, baud=baud, vehicle_class=MyVehicle)
        self.vehicle = connect(serial_address, wait_ready=True, baud=baud)
        self.thrust = 0.5
        self.default_alt = 1.0
        self.range_alt = 0.1
        # disable GPS	
        self.set_parameter('GPS_TYPE', 0)
        self.set_parameter('AHRS_EKF_TYPE', 3)
        self.set_parameter('EK2_ENABLE', 0)
        self.set_parameter('EK3_ENABLE', 1)
        self.set_parameter('AHRS_GPS_USE')
        self.set_parameter('FS_THR_ENABLE')
        # Use barometer
        self.set_parameter('EK3_SRC1_POSZ', 1)
        print("GPS: ", self.vehicle.gps_0)

    def set_parameter(self, param, value=0):
        print("Write vehicle param", param, ": ", self.vehicle.parameters[param], "to",  value)
        self.vehicle.parameters[param]=value
        for x in range(1,5):
            #Callbacks may not be updated for a few seconds
            if self.vehicle.parameters[param]==value:
                print("Success!")
                break
            time.sleep(1)
        

    
    def arm_nogps(self):
        self.init_yaw = math.degrees(self.vehicle.attitude.yaw)
        '''
        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        # If you need to disable the arming check,
        # just comment it with your own responsibility.
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)
        '''

        print("Arming motors")
        # Copter should arm in GUIDED_NOGPS mode
        self.vehicle.mode = VehicleMode("GUIDED_NOGPS")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)
        print("Arming Ready!")
        time.sleep(3)
        

    def takeoff_nogps(self, aTargetAltitude):
        ##### CONSTANTS #####
        DEFAULT_TAKEOFF_THRUST = 0.6
        SMOOTH_TAKEOFF_THRUST = 0.55

        print("Taking off!")

        thrust = DEFAULT_TAKEOFF_THRUST
        start = time.time()
        while True:
            # current_altitude = self.vehicle.location.global_relative_frame.alt
            current_altitude = self.vehicle.rangefinder.distance
            print(" Altitude: %f  Desired: %f" %
                  (current_altitude, aTargetAltitude))
            if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
                print("Reached target altitude. Time:", time.time()-start)
                break
            elif current_altitude >= aTargetAltitude*0.9:
                thrust = SMOOTH_TAKEOFF_THRUST

            self.send_attitude_target(roll_angle=0, pitch_angle=0, thrust = thrust, takeoff=True)
            # self.send_attitude_target(roll_angle=0, pitch_angle=0, yaw_angle=0, use_yaw_rate=False, thrust = thrust)
            if time.time()-start>8:
                print("Fail to Takeoff!")
                self.land()
                raise ValueError("Fail to Takeoff!")

            time.sleep(0.05)

    def arm_and_takeoff_nogps(self, aTargetAltitude):
        self.arm_nogps()
        self.takeoff_nogps(aTargetAltitude)
        """
        Arms vehicle and fly to aTargetAltitude without GPS data.
        """

    def send_attitude_target(self, roll_angle = 0.0, pitch_angle = 0.0,
                             yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = True,
                             thrust = None, takeoff=False):
        """
        use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                      When one is used, the other is ignored by Ardupilot.
        thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
                Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
                the code for maintaining current altitude.
        """
        ## Altitude Control
        """
        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        """
        if thrust == None or np.isnan(thrust):
            # current_altitude = self.vehicle.location.global_relative_frame.alt
            current_altitude = self.vehicle.rangefinder.distance
            # print("alt:", current_altitude)
            if current_altitude < self.default_alt-self.range_alt:
                # self.thrust = 0.55
                thrust = 0.55
            elif current_altitude > self.default_alt+self.range_alt:
                # self.thrust = 0.45
                thrust = 0.45
            else:
                # self.thrust = 0.5
                thrust = 0.5
        
        ## Yaw Control <use_yaw_rate==True>
        if takeoff:
            yaw_angle = self.init_yaw
        elif use_yaw_rate:
            yaw_angle = math.degrees(self.vehicle.attitude.yaw)
            #print('yaw_angle:', yaw_angle)
        else:
            yaw_angle += self.init_yaw
        '''
        if use_yaw_rate:
            yaw_rate = yaw_rate*10
            # yaw_angle = self.vehicle.attitude.yaw + yaw_rate
            yaw_angle = math.degrees(self.vehicle.attitude.yaw)
            print('yaw_angle:', yaw_angle)
        else:
            if yaw_angle is None:
                # this value may be unused by the vehicle, depending on use_yaw_rate
                yaw_angle = self.vehicle.attitude.yaw
                velocity = np.dot(np.array([[-math.sin(yaw_angle), math.cos(yaw_angle), 0],
                                    [math.cos(yaw_angle), math.sin(yaw_angle), 0], 
                                    [0, 0, 1]]), np.array(self.vehicle.velocity))
                yaw_angle = math.degrees(yaw_angle)
                print(velocity, yaw_angle)
            else:
                yaw_angle += self.vehicle.attitude.yaw
                yaw = self.vehicle.attitude.yaw
                velocity = np.dot(np.array([[-math.sin(yaw), math.cos(yaw), 0],
                                    [math.cos(yaw), math.sin(yaw), 0], 
                                    [0, 0, 1]]), np.array(self.vehicle.velocity))
                yaw = math.degrees(yaw)
                print(yaw)
                # print(cvel:', velocity, 'yaw:', yaw)
        '''
        
        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0, # time_boot_ms
            1, # Target system
            1, # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            self.to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
            0, # Body roll rate in radian
            0, # Body pitch rate in radian
            math.radians(yaw_rate), # Body yaw rate in radian/second
            thrust  # Thrust
        )
        self.vehicle.send_mavlink(msg)

    def foreward(self, foreward_times):
        while foreward_times>0:
            self.send_attitude_target(roll_angle=0, pitch_angle=-0.2, yaw_angle=self.init_yaw, use_yaw_rate=False)
            time.sleep(0.01)
            print("foreward...")
            foreward_times-=1
            
    def set_attitude(self, roll_angle = 0.0, pitch_angle = 0.0,
                     yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                     thrust = 0.5, duration = 0):
        """
        Note that from AC3.3 the message should be re-sent more often than every
        second, as an ATTITUDE_TARGET order has a timeout of 1s.
        In AC3.2.1 and earlier the specified attitude persists until it is canceled.
        The code below should work on either version.
        Sending the message multiple times is the recommended way.
        """
        self.send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        start = time.time()
        while time.time() - start < duration:
            self.send_attitude_target(roll_angle, pitch_angle,
                                 yaw_angle, yaw_rate, False,
                                 thrust)
            time.sleep(0.1)
        # Reset attitude, or it will persist for 1s more due to the timeout
        self.send_attitude_target(0, 0,
                             0, 0, True,
                             thrust)

    def to_quaternion(self, roll = 0.0, pitch = 0.0, yaw = 0.0):
        """
        Convert degrees to quaternions
        """
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w, x, y, z]
        
    def land(self):
        self.vehicle.mode = VehicleMode("LAND")
        while True:
            if self.vehicle.mode == "LAND":
                print("drone height: ",self.vehicle.rangefinder.distance)
                if self.vehicle.rangefinder.distance<0.12:
                    print("force disarmed")
                    self.force_disarmed()
                    break
            else:
                self.vehicle.mode = VehicleMode("LAND")
            
            time.sleep(0.1)
    
    def force_disarmed(self):
        msg = self.vehicle.message_factory.command_long_encode(0, 0, 400, 0, 0, 21196, 0, 0, 0, 0, 0)
        self.vehicle.send_mavlink(msg)

    def get_altitude(self):
        return self.vehicle.rangefinder.distance
'''
    def reboot(self):
        """Requests an autopilot reboot by sending a ``MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`` command."""

        reboot_msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,  # command
            0,  # confirmation
            1,  # param 1, autopilot (reboot)
            0,  # param 2, onboard computer (do nothing)
            0,  # param 3, camera (do nothing)
            0,  # param 4, mount (do nothing)
            0, 0, 0)  # param 5 ~ 7 not used

        self.vehicle.send_mavlink(reboot_msg)
        '''

if __name__ == "__main__":
    drone = Drone(serial_address='/dev/ttyTHS0', baud=57600)

    try: 
        drone.arm_and_takeoff_nogps(1.0)
        time.sleep(10)

        drone.land()
    except Exception as e:
        print(e)
    finally:
        drone.land()
        drone.vehicle.close()
