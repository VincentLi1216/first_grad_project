from cmath import nan
import collections, numpy as np, cv2, datetime
import multiprocessing as mp
import matplotlib.pyplot as plt
from pid_controller.pid import PID
from drone_control_nogps import Drone
import time, queue, sys, threading
from movement import Movement, State, Task
from track import TrackProcess

class AvoidProcessing:
    
    def __init__(self, gstream_def = None, simulate = False):
        self.process_running = queue.Queue()
        ## 實體化 TrackProcess ObstacleProcess Drone

        self.obstacle_q = mp.Queue(1)
        self.tracking_q = mp.Queue(1) # (roll, pitch, yaw)
        self.obstacle_img_q = mp.Queue(1)
        self.tracking_img_q = mp.Queue(1)

        self.video_deque = collections.deque(
            [np.zeros((800, 450, 3), dtype='uint8')], maxlen=1
        )

        print("Initialize track/obstacle process...", end="\t")
        if simulate:
            self.drone = Drone(serial_address='udp:127.0.0.1:14550', baud=57600)
            self.track = TrackProcess(  simulate = simulate,
                                        tracking_q = self.tracking_q )
        else:
            self.drone = Drone(serial_address='/dev/ttyTHS1', baud=57600)
            self.track = TrackProcess(  gstream_def = gstream_def,
                                        simulate = simulate,
                                        tracking_q = self.tracking_q,
                                        tracking_img_q = self.tracking_img_q)

        ## TrackingProcess and ObstacleProcess start
        self.track.start()

        from obstacle import ObstacleProcess
        self.obstacle = ObstacleProcess(obstacle_q = self.obstacle_q,
                                        obstacle_img_q = self.obstacle_img_q)
        self.obstacle.start()


        ## stop signal
        self.exception_list = queue.Queue()
        self.thread_stop = False
        
        ## control
        self.last_control = (0, 0, 0)
        self.rollPID = PID(p=0.008, i=0, d=0) # p=0.004, i=0.0001, d=0.0003  # p=0.005 #1st try 0.006 0.01
        self.pitchPID = PID(p=0.008, i=0, d=0) # p=0.004, d=0.0001   # p=0.005 #1st try 0.006 0.01
        self.yawPID = PID(p=1.7)   # p=10
        self.rollErr = []
        self.x_err = []
        self.roll_record = 0.0
        self.pitchErr = []
        self.yawErr = []
        self.t = []
        # self.start_time = time.time()
        
        self.buffer_size = 3
        self.controls_buffer = collections.deque(
            self.buffer_size * [(0.0, 0.0, 0.0, np.nan)], self.buffer_size
        )
        self.controls_display = collections.deque(
            [(0.0, 0.0, 0.0, np.nan)],1
        )

        ## related to avoidance
        self.state = State.tracking
        self.task = Task.start
        self.prev_task = Task.start

        self.prev_trackingDir = collections.deque(
            self.buffer_size * [(0.0, 0.0, 0.0)], self.buffer_size
        )
        
        self.avoid_height = self.drone.default_alt # used to record the height of current situation
        self.default_height = self.drone.default_alt # used to record the height of normal situation
        self.max_height = 4.0
        self.past_height = self.default_height
        
        self.slow_dwon = 0.3
        self.count = 0
        self.brake_count = 0

        ## video record initial
        self.record_start = False
        video_fps = 20
        self.video_fps = video_fps
        frameSize = (800, 450)
        self.date_str = datetime.datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
        self.video_recording = cv2.VideoWriter(
            "live_processing_front_{}.avi".format(self.date_str),
            cv2.VideoWriter_fourcc(*"DIVX"),
            video_fps,
            frameSize,
        )

        ## show img deque
        self.processing_img = collections.deque([np.zeros((212, 120, 3), dtype='uint8')], maxlen=1)

        ## run and show img thread start
        self.obstacleDetected = np.array(self.obstacle_q.get())
        self.obstacle_img = self.obstacle_img_q.get()
        self.trackingDir = self.tracking_q.get() # (roll, pitch, yaw)
        self.tracking_img = self.tracking_img_q.get()

        self.t_run = threading.Thread(target=self._run)
        self.t_run.daemon = True
        self.t_run.start()

        self.t_show_img = threading.Thread(target=self._show_img)
        # self.t_show_img.daemon = True
        self.t_show_img.start()
        print("video process init ready!")

        self.t_img = threading.Thread(target=self._save_img)
        self.t_img.daemon = True
        self.t_img.start()

        self.start_time = time.time()
        self.process_running.put("[process]init ready")
              
    def _show_img(self):
        while True:
            if self.thread_stop:
                cv2.destroyAllWindows()
                print("[process]break _show_img thread")
                break
            try:
                frame = self.processing_img.pop()
            except IndexError:
                continue
            cv2.imshow("foreward-camera", frame)
            cv2.waitKey(1)
            
                
    def show_save_err_plt(self):

        with open(f'log {self.date_str}.npy', 'wb') as f:
            np.save(f, self.t)
            np.save(f, self.rollErr)
            np.save(f, self.x_err)
            f.close()

        plt.plot(self.t, self.rollErr)
        plt.savefig(f'roll {self.date_str}.png')
        # plt.show()
        plt.close()
        plt.plot(self.t, self.x_err)
        plt.savefig(f'xerr_4 {self.date_str}.png')
        # plt.show()
        plt.close()
        plt.plot(self.t, self.x_err)
        plt.plot(self.t, self.rollErr)
        plt.savefig(f'both {self.date_str}.png')
        # plt.show()
        plt.close()

#        plt.plot(self.t, self.pitchErr)
#        plt.savefig(f'pitch {self.date_str}.png')
#        # plt.show()
#        plt.close()
#        plt.plot(self.t, self.yawErr)
#        plt.savefig(f'yaw {self.date_str}.png')
#        # plt.show()
#        plt.close()
        
  
    def _run(self):
        try:
            while not self.thread_stop:
                # self.test()
                
                if self.get_result():
                    
                    print("[process]state:", self.state)
                    
                    
                    if self.state == State.tracking:
                        avoid_strategy = self.obstacle_kind()

                    elif self.state == State.alert:
                        avoid_strategy = self.alert_kind()

                    elif self.state == State.avoid:
                        avoid_strategy = self.avoid_kind()

                    elif self.state == State.landing:
                        print('[process] force land')
                        self.drone.land()
                    
                    # control = self.control_normalize(self.trackingDir)
                    fcontrol = self.generate_control(self.trackingDir, avoid_strategy)
                    self.controls_buffer.appendleft(fcontrol)

                    # ctl = self.last_control
                    obstacle_img = self.obstacle_img.copy()
                    track_img = self.tracking_img.copy()
                    obstacle_img = cv2.resize(obstacle_img, (track_img.shape[1], track_img.shape[0]), interpolation=cv2.INTER_CUBIC)
                    text_img = np.zeros([track_img.shape[0], track_img.shape[1] + obstacle_img.shape[1], 3], dtype=np.uint8)
                    text_img.fill(255)
                    merged_img = np.concatenate((obstacle_img, track_img), axis=1)
                    merged_img = np.concatenate((merged_img, text_img), axis=0)
                    display_task = self.task if self.state == State.avoid else "---"
                    if self.state == State.alert:
                        cv2.putText(merged_img, f'state: {self.state}', (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))
                    else:
                        cv2.putText(merged_img, f'state: {self.state}', (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0))
                    cv2.putText(merged_img, f'task: {display_task}', (50, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0))
                    cv2.putText(merged_img, f'movement: {avoid_strategy}', (50, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0))
                    cv2.putText(merged_img, f'default_height: {self.avoid_height}', (50, 340), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0))
                    cv2.putText(merged_img, f'row, pitch, yaw, thrust: {round(self.controls_display[0][0], 3)}, {round(self.controls_display[0][1], 3)}, {round(self.controls_display[0][2], 3)}, {self.controls_display[0][3]}', (50, 370), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0))
                    
                    self.processing_img.appendleft(merged_img)
                    self.video_deque.appendleft(merged_img)
                    self.merged_img = merged_img
                    # time_elapsed = time.time() - prev
                    # print(time_elapsed)
                    # if time_elapsed > 1./30:
                    #     # store image
                    #     prev = time.time()
                    #     self.video_recording.write(merged_img)
                    # print("[process]time: ", time.time() - self.start_time)
                    # self.start_time = time.time()
                else:
                    continue
                    # print('in processing cant read image')
                print(avoid_strategy)
        except Exception as e:
            print("[process]_run thread recieve error: ", e)
            self.exception_list.put(sys.exc_info())
        # finally:
        #     if self.thread_stop:
        #         print("break--run")
        #         break
        finally:
            print("[process]break _run thread")


    def _save_img(self):
        while not self.thread_stop:
            # time.sleep(1./20)
            # prev = time.time()
            self.video_recording.write(self.video_deque[0])
            # print("video write time: ", time.time() - prev)


    def get_result(self):
        
        if not self.obstacle_q.empty() and not self.tracking_q.empty() \
            and not self.obstacle_img_q.empty() and not self.tracking_img_q.empty():
            self.obstacleDetected = np.array(self.obstacle_q.get())
            self.obstacle_img = self.obstacle_img_q.get()
            self.trackingDir = self.tracking_q.get() # (roll, pitch, yaw)
            self.tracking_img = self.tracking_img_q.get()
            print('[process]obstacleDetected', self.obstacleDetected)
            print('[process]trackingDir', self.trackingDir)
            return True
        else:
            return False

    def obstacle_kind(self):
        
        if len(self.obstacleDetected) == 0:
            return Movement.tracking

        elif not (0 in self.obstacleDetected[:,1]):
            # all in middle of farther
            return Movement.slow_tracking

        else:
            # near
            if len(np.argwhere((self.obstacleDetected[:,0]==-1) &
             (self.obstacleDetected[:,1]==0)))> 0:
                # print("there is undefined obstacle")
                self.state = State.alert
                self.brake_count = 0
                return Movement.braking

            else:
                # print("there is obstacle")
                self.state = State.avoid
                self.task = Task.start
                self.brake_count = 0
                return Movement.braking

    def alert_kind(self):
        # hover
        if self.brake_count <= 8:
            self.brake_count += 1
            return Movement.braking

        if len(self.obstacleDetected) == 0 or (not 0 in self.obstacleDetected[:,1]):
            # no obstacle
            self.state = State.tracking
            return Movement.last_forward
            
        if len(np.argwhere((self.obstacleDetected[:,0]==-1) &
             (self.obstacleDetected[:,1]==0))) == 0:
        #     # unidentified obstacle near UAV
        #     print("[process]unidentified obstacle near UAV")
        # else:
            # identified obstacle near UAV
            self.state = State.avoid
            self.task = Task.start
        self.brake_count +=1
        if self.brake_count >30:
            self.state = State.landing

        return Movement.hover


    def avoid_kind(self):
                
        if self.brake_count <= 7:
            self.brake_count += 1
            return Movement.braking
            
        if len(self.obstacleDetected) != 0:
            if len(np.argwhere((self.obstacleDetected[:,0]==0) &
                (self.obstacleDetected[:,1]==0))) > 0:
                #近距離有human
                if self.task != Task.hover:
                    self.prev_task = self.task
                self.task = Task.hover
                return Movement.hover
            else:
                if self.task == Task.start:
                    self.task  = Task.up
        
        height = self.drone.get_altitude() #self.test_height()
        if self.task == Task.up:
            # if there is no obstacle
            if len(self.obstacleDetected) == 0 or (not (0 in self.obstacleDetected[:,1])):
                # record current height
                self.avoid_height = height + 0.3
                self.drone.default_alt = height + 0.3

                self.task = Task.forward_toforward
                return Movement.first_forward
            else:
                if height > self.max_height:
                    self.state = State.landing
                    return Movement.land
                else:
                    return Movement.up
        # first step of forward (enter the second step if it UAV is above the obstacle)
        elif self.task == Task.forward_toforward:
            
            if height < self.default_height:
                self.count += 1
            else:
                self.count = 0
            if self.count > 3:
                self.drone.default_alt = 0.6
                self.avoid_height = 0.6
                self.task = Task.forward_todown
                self.count = 0
            return Movement.forward
            
        
        elif self.task == Task.forward_todown:
            
            if height > self.default_height:
                self.count += 1
            else:
                self.count = 0

            if self.count > 2:
                self.drone.default_alt = self.default_height
                self.avoid_height = self.default_height
                self.state = State.tracking
                return Movement.last_forward

            return Movement.forward
            

        elif self.task == Task.hover:
            if self.prev_task != Task.start:
                self.task = self.prev_task
                return Movement.hover
            else:
                if len(self.obstacleDetected) == 0:
                    self.state = State.tracking
                    return Movement.last_forward
                elif 0 in self.obstacleDetected[:,1]:
                    if len(np.argwhere((self.obstacleDetected[:,0]==-1) &
                        (self.obstacleDetected[:,1]==0))) > 0:
                        # unidentify obstacle near
                        self.state = State.alert
                        return Movement.hover
                    else:
                        # identify obstacle near
                        self.task = Task.up
                        return Movement.hover
                else:
                    # obstacle middle
                    self.state = State.tracking
                    return Movement.tracking
     
    def generate_control(self, control, strategy):
        roll, pitch, yaw = control
        self.roll_record = roll/6.67
        if roll != 0: # there is line
            self.prev_trackingDir.appendleft(control)
        else:   # there is no line
            prev_roll, prev_pitch, prev_yaw = np.mean(self.prev_trackingDir, axis = 0)

        if strategy == Movement.tracking:
            return (roll, pitch, yaw, np.nan)
        elif strategy == Movement.slow_tracking:
            return (roll, pitch * self.slow_dwon, yaw, np.nan)
        elif strategy == Movement.braking:
            print("[process] movement: ", strategy)
            return (roll, 700.0, 0.0, np.nan)
        elif strategy == Movement.hover:
            return (roll, 0.0, yaw, 0.5)
        elif strategy == Movement.first_forward:
            print("[process] movement: ", strategy)
            return (roll, pitch * 1.5, yaw, np.nan)
        elif strategy == Movement.forward:
            if roll != 0:
                return (roll, pitch, yaw, np.nan)
            else:
                return (prev_roll, prev_pitch, prev_yaw, np.nan)
        elif strategy == Movement.last_forward:
            print("[process] movement: ", strategy)
            return (roll, pitch * 1.5, yaw, np.nan)
        elif strategy == Movement.up:
            return (roll, 0.0, yaw, 0.55)

    # def control_normalize(self, control):
    #     if len(control)==2:
    #         x_err, yaw_err = control
    #         y_err = 0
    #     else:
    #         x_err, y_err, yaw_err = control
    #     return (x_err, y_err, yaw_err)
  
    def get_control(self):
        latest_control = np.mean(self.controls_buffer, axis = 0)
        latest_control[0] = self.rollPID(latest_control[0])
        latest_control[1] = self.pitchPID(latest_control[1])
        latest_control[2] = self.yawPID(latest_control[2])
        # if isnanself.controls_buffer[0][3]:
        #     latest_control[3] = self.controls_buffer[0][3]
        # print("[main] controls_buffer[0][3]: ", self.controls_buffer[0][3])
        latest_control[3] = None if np.isnan(self.controls_buffer[0][3]) else self.controls_buffer[0][3]
        # print("[main] latest_control[3]: ", latest_control[3])
        self.x_err.append(self.roll_record)
        self.rollErr.append(latest_control[0])
        self.pitchErr.append(latest_control[1])
        self.yawErr.append(latest_control[2])
        self.t.append(time.time()-self.start_time)

        self.last_control = latest_control
        self.controls_display.appendleft(latest_control)
        return latest_control

    def release(self):
        
        self.thread_stop = True
        self.t_img.join()
        self.video_recording.release()
        print("[process]video released sucess :D finally")
        self.t_run.join()
        self.t_show_img.join()
        # self.track.release()
        self.track.join()
        # self.obstacle.release()
        self.obstacle.join()
        print("[process]release success!")

    def test(self):
        obstacleKind = input("obstacle kind: ")
        obstacleKind = np.array(obstacleKind.split())
        obstacleDepth = input("depth label ")
        obstacleDepth = obstacleDepth.split()
        print(len(obstacleDepth))
        self.obstacleDetected = np.concatenate((obstacleKind, obstacleDepth), axis=0)
        print(self.obstacleDetected)

    def test_height(self):
        return float(input("input height: "))
 
