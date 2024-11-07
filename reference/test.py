import pyzed.sl as sl
import time
from pytorch_yolov5.reference.hand_eye_calibration import HandEyeCalibration

def main():
    zed = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 30
    zed.open(init)

    obj_runtime_param = sl.ObjectDetectionParameters()
    obj_set = sl.Objects()
    runtime_params = sl.RuntimeParameters()

    calibration = HandEyeCalibration()  # 初始化手眼標定

    while True:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_objects(obj_set, obj_runtime_param)

            # 獲取相機位置
            camera_position = zed.get_position()

            for obj in obj_set.object_list:
                label = obj.label
                confidence = obj.probability
                calibration.add_data(camera_position, label, confidence)  # 添加數據
            
            # 每10秒進行一次標定
            if int(time.time()) % 10 == 0:
                calibration_result = calibration.calibrate()
                if calibration_result is not None:
                    print(f"Calibration Data: {calibration_result}")

    zed.close()

if __name__ == "__main__":
    main()
