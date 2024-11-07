#!/usr/bin/env python3
import yaml
import sys
import numpy as np
import torch
import cv2
import pyzed.sl as sl
import torch.backends.cudnn as cudnn
from threading import Lock, Thread
import time
from robot_control import RobotArm  

sys.path.insert(0, './yolov5')
from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_boxes, xyxy2xywh
from utils.torch_utils import select_device
from utils.augmentations import letterbox

class ObjectDetector:
    def __init__(self):
        self.lock = Lock()
        self.run_signal = False
        self.exit_signal = False
        self.image_net = None
        self.detections = None
        self.detected_objects = []
        self.robot = RobotArm()
        self.robot.reset_position()
        
    def img_preprocess(self, img, device, half, net_size):
        net_image, ratio, pad = letterbox(img[:, :, :3], net_size, auto=False)
        net_image = net_image.transpose((2, 0, 1))[::-1]
        net_image = np.ascontiguousarray(net_image)

        img = torch.from_numpy(net_image).to(device)
        img = img.half() if half else img.float()
        img /= 255.0

        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        return img, ratio, pad

    def detections_to_custom_box(self, detections, im, im0):
        output = []
        for i, det in enumerate(detections):
            if len(det):
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()
                gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]

                for *xyxy, conf, cls in reversed(det):
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()
                    obj = sl.CustomBoxObjectData()
                    obj.bounding_box_2d = self.xywh2abcd(xywh, im0.shape)
                    obj.label = int(cls)
                    obj.probability = conf
                    obj.is_grounded = False
                    output.append(obj)
        return output

    def xywh2abcd(self, xywh, im_shape):
        output = np.zeros((4, 2))
        x_min = (xywh[0] - 0.5 * xywh[2]) * im_shape[1]
        x_max = (xywh[0] + 0.5 * xywh[2]) * im_shape[1]
        y_min = (xywh[1] - 0.5 * xywh[3]) * im_shape[0]
        y_max = (xywh[1] + 0.5 * xywh[3]) * im_shape[0]
        
        output[0] = [x_min, y_min]
        output[1] = [x_max, y_min]
        output[2] = [x_max, y_max]
        output[3] = [x_min, y_max]
        return output

    def torch_thread(self, weights, img_size, conf_thres=0.2, iou_thres=0.45):
        print("初始化網路...")
        device = select_device()
        half = device.type != 'cpu'
        imgsz = check_img_size(img_size)

        model = attempt_load(weights, device=device)
        if half:
            model.half()
        
        if device.type != 'cpu':
            model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))

        while not self.exit_signal:
            if self.run_signal:
                with self.lock:
                    img, ratio, pad = self.img_preprocess(self.image_net, device, half, imgsz)
                    pred = model(img)[0]
                    det = non_max_suppression(pred, conf_thres, iou_thres)
                    self.detections = self.detections_to_custom_box(det, img, self.image_net)
                self.run_signal = False
            time.sleep(0.01)

    def start_detection(self, weights='./yolov5/yolov5s.pt', img_size=416, conf_thres=0.4):
        print("初始化相機...")
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.coordinate_units = sl.UNIT.METER
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.depth_maximum_distance = 50
        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            print("無法開啟相機")
            return False

        # 啟用位置追蹤
        positional_tracking_parameters = sl.PositionalTrackingParameters()
        self.zed.enable_positional_tracking(positional_tracking_parameters)

        # 啟用物體偵測與分割
        obj_param = sl.ObjectDetectionParameters()
        obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        obj_param.enable_tracking = True
        obj_param.enable_segmentation = True  # 新增分割功能
        self.zed.enable_object_detection(obj_param)

        # 初始化物體檢測執行緒
        self.detection_thread = Thread(target=self.torch_thread, 
                                     kwargs={'weights': weights, 
                                           'img_size': img_size, 
                                           'conf_thres': conf_thres})
        self.detection_thread.start()

        return True

    def get_detected_objects(self):
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            with self.lock:
                self.zed.retrieve_image(self.image_net, sl.VIEW.LEFT)
            self.run_signal = True

            while self.run_signal:
                time.sleep(0.001)

            objects = sl.Objects()
            with self.lock:
                self.zed.ingest_custom_box_objects(self.detections)
            self.zed.retrieve_objects(objects)

            detected_objects = []
            for obj in objects.object_list:
                if obj.tracking_state == sl.OBJECT_TRACKING_STATE.OK:
                    position = obj.position
                    print(f"檢測到物體，位置：x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}")
                    success = self.robot.grab_object([position[0], position[1], position[2]])
                    if success:
                        print(f"成功抓取物體")
                    else:
                        print(f"抓取失敗")

                    detected_objects.append({
                        'position': obj.position,
                        'label': obj.label,
                        'confidence': obj.confidence
                    })
            
            return detected_objects
        return []

    def stop(self):
        self.exit_signal = True
        if hasattr(self, 'detection_thread'):
            self.detection_thread.join()
        if hasattr(self, 'zed'):
            self.zed.close()
        self.robot.reset_position()
        self.robot.close()

if __name__ == '__main__':
    detector = ObjectDetector()
    if detector.start_detection():
        try:
            while True:
                objects = detector.get_detected_objects()
                for obj in objects:
                    print(f"檢測到物體：位置={obj['position']}, 類別={obj['label']}, 信心度={obj['confidence']}")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n程序被使用者中斷")
        finally:
            detector.stop()
