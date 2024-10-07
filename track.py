from multiprocessing import Process, Queue, Pipe #, Event
import numpy as np, math, cv2
import queue, traceback, signal


class TrackProcess(Process):
    def __init__(self, gstream_def = None, simulate = False, 
                    tracking_q = None, tracking_img_q = None):
        
        super(TrackProcess, self).__init__()
        print("Initialize tracking pipeline...", end="\t")
    
        self.width = 400
        self.height = 225

        self.simulate = simulate
        self.gstream_def = gstream_def

        self.tracking_q = tracking_q
        self.tracking_img_q = tracking_img_q

        self._pconn, self._cconn = Pipe()   # p: parent  c: child

    def process_init(self):

        print("downward camera --init start")
        if self.simulate:
            from utils.video_capture_sim import VideoCapture
            self.cap = VideoCapture()
        else:
            from utils.video_capture_UAV import VideoCapture
            self.cap = VideoCapture(self.gstream_def, cv2.CAP_GSTREAMER)
        print(self.cap.isOpened())
        if not self.cap.isOpened():
            raise ValueError("The downward camera is not open.")
        
        self.thread_stop = False
        print("downward camera --init ready")

    def run(self):

        try: 
            
            self.process_init()

            while (not self.thread_stop):

                # if self.stop.is_set():
                #     print('[track]stop.is_set')
                #     self.thread_stop = True
                #     break

                # Get image
                re, image = self.cap.read()
                if re:
                    new_img = image.copy()
                    ctl = self.tracking(new_img)
                    # print("in process: ", ctl)
                    if not self.tracking_q.empty():
                        try:
                            self.tracking_q.get_nowait() # discard previous (unprocessed) frame
                        except queue.Empty:
                            pass
                    self.tracking_q.put(ctl)

                    cv2.putText(new_img, f'x:{ctl[0]}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255))
                    cv2.putText(new_img, f'y:{ctl[1]}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255))
                    cv2.putText(new_img, f'yaw:{ctl[2]}', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255))
                    
                    # if self.record_start:
                    #     self.video_recording.write(image)
                    # self.processing_img.appendleft(image)

                    if not self.tracking_img_q.empty():
                        try:
                            self.tracking_img_q.get_nowait() # discard previous (unprocessed) frame
                        except queue.Empty:
                            pass
                    self.tracking_img_q.put(new_img)
                else:
                    # raise ValueError("The downward camera is not open.")
                    print("[track] There is no image D:")

        except KeyboardInterrupt:
            print("[track]recieve keyboardinterrupt")
        
        except Exception as e:
            tb = traceback.format_exc()
            self._cconn.send((e, tb))
            self.thread_stop = True
            print("[track]exception catch")
            print(e, tb)

        finally:
            print("[track]break _run")
            self.cap.release()
            print("[track]resources release in track")
   

    def tracking(self, image):
        # hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        th1 = np.zeros((self.height, self.width), dtype='uint8')
        th1[(image[:, :, 2]>115) & (image[:, :, 0]<105) & (image[:, :, 1]<95)] = 255
        kernel = np.ones((5, 5), np.uint8)
        th1 = cv2.morphologyEx(th1, cv2.MORPH_OPEN, kernel)
        
        # print("next_fstate: ", self.next_fstate)
        
        ## <Line> Fit ellipse ##
        pts = np.nonzero(th1==255)
        if len(np.argwhere(th1==255))<300:
            # print("Can't find tracking line")
            return (0,0,0)
        (x,y),(MA,ma),angle = cv2.fitEllipse(np.transpose((pts[1], pts[0])))
        
        cv2.ellipse(image,((x,y),(MA,ma),angle),(0,255,0),2)
        x_err = x-self.width/2
        
        x_err = (1/(1+math.pow(math.e,-(x_err/40))) - 0.5) * 400
        if abs(angle) > 180:
            return (0, 0, 0)
        if angle>90:
            angle = angle-180
        
        yaw_err=angle*(math.log(abs(angle)+1, 1.8))
        # yaw_err=angle
        '''for curve
        if abs(angle)>4:
            angle = math.radians(angle)
            phi = math.radians(40) if angle > 0 else math.radians(-40)
            y_err = -(100-2*angle)*math.cos(phi+angle)
            x_err += (128-2*angle)*math.sin(phi+angle)
            # x_err = 0
            # y_err=-(17-abs(angle)*0.018)
            return x_err, y_err, yaw_err
        '''
        return (x_err, -90, yaw_err)

    def release(self):
        print('[track]release by process')
        # self.stop.set()

    @property
    def exception(self):
        if self._pconn.poll(): # there is value in _pconn
            return self._pconn.recv()
        return False

if __name__ == '__main__':
    
    
    def signal_handler(signal_num, frame):
        print("handleing")
        print(signal_num)
        raise RuntimeError("system quit")

    signal.signal(signal.SIGQUIT, handler = signal_handler) # ctrl + c
    signal.signal(signal.SIGINT, handler = signal_handler)  # ctrl + \


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
            flip_method=0,
            display_width=400,
            display_height=225,
        )
    )


    q = Queue(1)

    tr = TrackProcess(gstream_def = gstream_pipeline, simulate = False, 
                    dirqueue = q)

    print("init ready")
    tr.start()
    try:
        while True:
            print(q.get())
    except:
        print("except")
    finally:
        print("in finally")
        tr.release()
        tr.join()
