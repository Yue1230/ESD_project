import yaml
import sys
import numpy as np
import torch
import cv2
import pyzed.sl as sl
import time
from robotAtoB import RobotArm  # 引入 RobotArm 類別

# YOLO 和 ZED 相機相關匯入
sys.path.insert(0, './yolov5')
from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_boxes, xyxy2xywh
from utils.torch_utils import select_device
from utils.augmentations import letterbox

# 初始化機械手臂
robot_arm = RobotArm()

# 加載 COCO 類別名稱
with open('./yolov5/data/coco.yaml', 'r', encoding='utf-8') as f:
    coco_data = yaml.safe_load(f)
coco_classes = coco_data['names']

def img_preprocess(img, device, half, net_size):
    net_image, ratio, pad = letterbox(img[:, :, :3], net_size, auto=False)
    net_image = net_image.transpose((2, 0, 1))[::-1]
    net_image = np.ascontiguousarray(net_image)

    img = torch.from_numpy(net_image).to(device)
    img = img.half() if half else img.float()
    img /= 255.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)
    return img, ratio, pad

def detect_and_grab_object():
    # 初始化相機和物件偵測模型
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    zed.open(init_params)

    # 加載 YOLO 模型
    device = select_device()
    model = attempt_load('./yolov5/yolov5s.pt', map_location=device)
    model.eval()

    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # 取得影像並進行預處理
            img = zed.retrieve_image(sl.VIEW.LEFT)
            img, _, _ = img_preprocess(img, device, half=False, net_size=640)
            pred = model(img)[0]
            detections = non_max_suppression(pred, 0.4, 0.5)
            
            for det in detections:
                if len(det):
                    # 確認是否有偵測到水瓶 (COCO 類別 ID)
                    for *xyxy, conf, cls in det:
                        label = coco_classes[int(cls)]
                        if label == "bottle":  # 當偵測到「水瓶」
                            print("水瓶偵測到，開始執行夾取動作")
                            robot_arm.move_to_a_point_and_grab()  # 移動到 A 點夾取
                            robot_arm.move_to_b_point()  # 移動到 B 點放下
                            robot_arm.reset_position()  # 重置位置
                            break
            time.sleep(1)

if __name__ == '__main__':
    try:
        detect_and_grab_object()
    finally:
        robot_arm.close()
