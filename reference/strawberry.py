import serial
import time
import numpy as np
import cv2 as cv
import pyzed.sl as sl

# 初始化串口連接
arduino = serial.Serial('/dev/cu.usbserial-1120', 9600, timeout=1)

# 定義舵機號碼
shoulderServo1 = 0 
shoulderServo2 = 1 
elbowServo = 2     
wristServo = 3     
gripperServo = 4
baseServo = 5

# 設定單個舵機角度
def set_servo_angle(servo_num, angle):
    if 0 <= servo_num < 16:
        command = f"{servo_num},{angle}\n"
        arduino.write(command.encode())
        time.sleep(0.05)

# 定義影像處理區域
region_rows = 64
region_cols = 64

# 使用 ZED 2 相機初始化
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720  # 1280x720 resolution
init_params.camera_fps = 30  # FPS
zed.open(init_params)

def findStrawberry(bgr_image):
    rows, cols, _ = bgr_image.shape
    bgr_region = bgr_image[int(rows/2)-region_rows:int(rows/2)+region_rows,
                           int(cols/2)-region_cols:int(cols/2)+region_cols]
    
    img_hsv = cv.cvtColor(bgr_region, cv.COLOR_BGR2HSV)
    
    # 紅色遮罩 (0-10)
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv.inRange(img_hsv, lower_red, upper_red)
    # 深紅色遮罩
    lower_red = np.array([160,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv.inRange(img_hsv, lower_red, upper_red)
    maskRed = mask0 + mask1

    # 黃綠色遮罩
    lower_green = np.array([20,50,50])
    upper_green = np.array([60,255,255])
    maskGreen = cv.inRange(img_hsv, lower_green, upper_green)
    
    red_ratio = cv.sumElems(maskRed)[0] / 255 / region_rows / region_cols / 4
    green_ratio = cv.sumElems(maskGreen)[0] / 255 / region_rows / region_cols / 4

    cv.rectangle(bgr_image,
                 (int(cols/2)-region_cols, int(rows/2)-region_rows),
                 (int(cols/2)+region_cols, int(rows/2)+region_rows), (255, 0, 0), 3)

    cv.imshow('Camera', bgr_image)
    cv.imshow('maskRed', maskRed)
    cv.imshow('maskGreen', maskGreen)

    if red_ratio > 0.6:
        return 'red'
    elif green_ratio > 0.6:
        return 'green'
    else:
        return 'no strawberry'

# 從 ZED 2 捕獲影像
image = sl.Mat()
while True:
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.LEFT)
        frame = image.get_data()
        frame_bgr = cv.cvtColor(frame, cv.COLOR_RGBA2BGR)
        
        object_color = findStrawberry(frame_bgr)
        print(object_color)

        if object_color == 'no strawberry':
            continue
        
        # 控制手臂抓取
        if object_color == 'red':
            set_servo_angle(shoulderServo1, 90)  # 控制角度可以根據需要調整
            set_servo_angle(shoulderServo2, 90)
            set_servo_angle(elbowServo, 45)     # 模擬抓取動作
            time.sleep(1)
            set_servo_angle(gripperServo, 60)   # 夾取草莓
            time.sleep(1)
        elif object_color == 'green':
            set_servo_angle(shoulderServo1, 45)
            set_servo_angle(shoulderServo2, 135)
            set_servo_angle(elbowServo, 60)
            time.sleep(1)
            set_servo_angle(gripperServo, 60)
            time.sleep(1)

        # 返回初始位置
        set_servo_angle(shoulderServo1, 0)
        set_servo_angle(shoulderServo2, 180)
        set_servo_angle(elbowServo, 90)
        set_servo_angle(gripperServo, 90)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

# 釋放資源
zed.close()
cv.destroyAllWindows()
