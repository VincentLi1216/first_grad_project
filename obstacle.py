"""trt_yolo.py

This script demonstrates how to do real-time object detection with
TensorRT optimized YOLO engine.
"""


from concurrent.futures import thread
from distutils import dep_util
from multiprocessing import Process, Queue #, Event
import os
import time
import argparse
import threading, queue

import cv2
import numpy as np
import pycuda.autoinit  # This is needed for initializing CUDA driver
import pycuda.driver as cuda

from utils.yolo_classes_custom import get_cls_dict
from utils.camera_custom import Camera
from utils.display import open_window, set_display, show_fps
from utils.visualization import BBoxVisualization
from utils.yolo_with_plugins import TrtYOLO


WINDOW_NAME = 'TrtYOLODemo'

class ObstacleProcess(Process):
    def __init__(self, obstacle_q = None, obstacle_img_q = None):
        super(ObstacleProcess, self).__init__()
        self.img = None
        self.obstacle_q = obstacle_q
        self.obstacle_img_q = obstacle_img_q

        # self.stop = Event()

    def yolo_proccesing(self, trt_yolo, conf_th):
        """Continuously capture images from camera and do object detection.

        # Arguments
        cam: the camera instance (video source).
        trt_yolo: the TRT YOLO object detector instance.
        conf_th: confidence/score threshold for object detection.
        vis: for visualization.
        """
        full_scrn = False
        fps = 0.0
        tic = time.time()
        while not self.thread_stop:
            # if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
            #     break
            n, img = self.cam.read_color()
            if img is None:
                break
            self.img = img.copy()
            # boxes, confs, clss = trt_yolo.detect(img, conf_th)
            result = trt_yolo.detect(img, conf_th)
            if not self.yolo_result.empty():
                try:
                    self.yolo_result.get_nowait()  # discard previous (unprocessed) frame
                except queue.Empty:
                    pass
            self.yolo_result.put(result)
           
            '''
            img = vis.draw_bboxes(img, boxes, confs, clss)
            img = show_fps(img, fps)
            cv2.imshow(WINDOW_NAME, img)
            toc = time.time()
            curr_fps = 1.0 / (toc - tic)
            # calculate an exponentially decaying average of fps number
            fps = curr_fps if fps == 0.0 else (fps*0.95 + curr_fps*0.05)
            tic = toc
            key = cv2.waitKey(1)
            if key == 27:  # ESC key: quit program
                break
            elif key == ord('F') or key == ord('f'):  # Toggle fullscreen
                full_scrn = not full_scrn
                set_display(WINDOW_NAME, full_scrn)
            '''


    def depth_processing(self):
        while not self.thread_stop:
            n, img = self.cam.read_depth()

            th1_far = np.uint8(np.where((img<2600) & (img!=0), 255, 0))
            contours, hierarchy = cv2.findContours(th1_far, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_far = list(filter(lambda x: cv2.contourArea(x)>300, contours))
            
            th1_near = np.uint8(np.where((img<1000) & (img!=0), 255, 0))
            contours, hierarchy = cv2.findContours(th1_near, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_near = list(filter(lambda x: cv2.contourArea(x)>300, contours))

            far_index = set()
            result = []
            for cnt_near in contours_near:
                pt = (np.uint8(cnt_near[0, 0, 0]),np.uint8(cnt_near[0, 0, 1]))
                for i, cnt_far in enumerate(contours_far):
                    if cv2.pointPolygonTest(cnt_far, pt, False)>=0:
                        if i not in far_index:
                            result.append([cnt_far, 0])
                            far_index.add(i)
                    else:
                        result.append([cnt_near, 0])
            for i, cnt_far in enumerate(contours_far):
                if i not in far_index:
                    result.append([cnt_far, 1])
            
            if not self.depth_result.empty():
                try:
                    self.depth_result.get_nowait()  # discard previous (unprocessed) frame
                except queue.Empty:
                    pass
            self.depth_result.put(result)


    def _init_yolo_inference(self):
        model = 'yolov4-tiny-custom'
        category_num = 4
        letter_box = False
        if not os.path.isfile('yolo/%s.trt' % model):
            raise SystemExit('ERROR: file (yolo/%s.trt) not found!' % model)
        cls_dict = get_cls_dict()
        self.vis = BBoxVisualization(cls_dict)
        cuda.init()
        ctx = cuda.Device(0).make_context()
        trt_yolo = TrtYOLO(model, category_num, letter_box, cuda_ctx = ctx)
        '''
        open_window(
            WINDOW_NAME, 'Camera TensorRT YOLO Demo',
            self.cam.width, self.cam.height)
        '''
        conf_thresh = 0.35
        thread_yolo = threading.Thread(target=self.yolo_proccesing, args=(trt_yolo, conf_thresh))
        thread_yolo.daemon=True
        thread_yolo.start()


    def _init_depth_processing(self):
        thread_depth = threading.Thread(target=self.depth_processing)
        thread_depth.daemon=True
        thread_depth.start()

    def process_init(self):
        
        self.thread_stop = False
        self.cam = Camera()
        '''if not cam.isOpened():
            raise SystemExit('ERROR: failed to open camera!')'''
        self.yolo_result = queue.Queue(maxsize=1)
        self._init_yolo_inference()
        self.depth_result = queue.Queue(maxsize=1)
        self._init_depth_processing()
        self.result = queue.Queue(maxsize=1)


    def run(self):

        try:
            self.process_init()

            while not self.thread_stop:

                # if self.stop.is_set():
                #     print('[obstacle]stop.is_set')
                #     self.thread_stop = True
                #     break
                result = []
                depth_result = self.depth_result.get()
                boxes, confs, clss = self.yolo_result.get()
                img = self.img.copy()
                for (cnt, pos) in depth_result:
                    obstacle = -1
                    for bb, cf, cl in zip(boxes, confs, clss):
                        # x_min, y_min, x_max, y_max = bb[0], bb[1], bb[2], bb[3]
                        center = (np.uint8((bb[0]+bb[2])//2), np.uint8((bb[1]+bb[3])//2))
                        if cv2.pointPolygonTest(cnt, center, False)>=0:
                            if cl == 0 or obstacle == -1:
                                obstacle = np.int8(cl)
                    result.append([obstacle, pos])
                    # img <- near(red)/far(green) contours
                    if pos == 0: # near
                        img = cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)
                    else: # far
                        img = cv2.drawContours(img, [cnt], 0, (0, 255, 0), 3)
                if not self.result.empty():
                    try:
                        self.result.get_nowait()
                    except queue.Empty:
                        pass
                self.result.put(result)

                if not self.obstacle_q.empty():
                    try:
                        self.obstacle_q.get_nowait()
                    except queue.Empty:
                        pass
                self.obstacle_q.put(result)
                
                # img <- yolo box
                img = self.vis.draw_bboxes(img, boxes, confs, clss)
                # img -> ob_img_queue
                if not self.obstacle_img_q.empty():
                    try:
                        self.obstacle_img_q.get_nowait()
                    except queue.Empty:
                        pass
                self.obstacle_img_q.put(img)

                # print('result', result)
        except KeyboardInterrupt:
            print("[obstacle]recieve keyboardinterrupt")
        
        finally:    
            self.cam.release()
            print("[obstacle]cam released")
            # self.thread_depth.join()
            # self.thread_yolo.join()

    def release(self):
        print('[obstacle]release by process')
        # self.stop.set()


def main():
    # go = ObstacleProcess()
    # cv2.destroyAllWindows()
    q = Queue()
    ob = ObstacleProcess(obqueue = q)

    print("init ready")
    ob.start()
    time.sleep(5)
    for i in range(500):
        if not q.empty():
            print("in main: ", q.get())
        else:
            print("continue")
        time.sleep(0.05)

        # if tr.exception:
        #     error, traceback = tr.exception
        #     print(traceback)

    ob.release()
    ob.terminate()


if __name__ == '__main__':
    main()
