import serial
import time
import tkinter as tk

# 初始化串口連接，設置對應的COM口和波特率
arduino = serial.Serial('COM11', 9600, timeout=1)  # 確認串口名稱正確

# 定義舵機號碼
shoulderServo1 = 0 
shoulderServo2 = 1 
elbowServo = 2     
wristServo = 3     
gripperServo = 4
baseServo = 5

# 定義每個舵機的最小值和最大值
shoulder_min, shoulder_max = 0, 180     # 肩膀舵機 (0和1的範圍相同)
elbow_min, elbow_max = 20, 180          # 手肘舵機 (舵機2)
wrist_min, wrist_max = 0, 180           # 手腕舵機 (舵機3)
gripper_min, gripper_max = 20, 100      # 夾爪舵機 (舵機4)
base_min, base_max = 0, 180         # 舵機5

# 設定單個舵機角度
def set_servo_angle(servo_num, angle):
    if 0 <= servo_num < 16:  # 最大支援16個舵機
        command = f"{servo_num},{angle}\n"
        arduino.write(command.encode())  # 發送控制指令給Arduino
        time.sleep(0.01)  # 等待Arduino處理

# 同時設定兩個舵機角度
def set_servo_angles(servo_1_num, angle_1, servo_2_num, angle_2):
    command_1 = f"{servo_1_num},{angle_1}\n"
    command_2 = f"{servo_2_num},{angle_2}\n"
    arduino.write(command_1.encode())  # 發送第一個舵機的控制指令
    arduino.write(command_2.encode())  # 發送第二個舵機的控制指令
    time.sleep(0.01)  # 等待Arduino處理

# 更新肩膀舵機的角度（A 與 B 相反旋轉）
def update_shoulder_angle(angle):
    angle = int(angle)
    # 正向旋轉：舵機A從0度轉到180度，舵機B從180度轉到0度
    set_servo_angle(shoulderServo1, angle)          # 舵機A從0度旋轉到180度
    set_servo_angle(shoulderServo2, 180 - angle)    # 舵機B從180度反向旋轉到0度

# 更新手肘舵機的角度
def update_elbow_angle(angle):
    set_servo_angle(elbowServo, angle)

# 更新手腕舵機的角度
def update_wrist_angle(angle):
    set_servo_angle(wristServo, angle)

# 更新夾爪舵機的角度
def update_gripper_angle(angle):
    set_servo_angle(gripperServo, angle)

# 重置所有舵機角度
def reset_all_servos():
    set_servo_angle(baseServo, 90)
    time.sleep(0.5)
    set_servo_angle(shoulderServo1, 90)
    set_servo_angle(shoulderServo2, 90)
    time.sleep(0.5)
    set_servo_angle(elbowServo, 90)
    time.sleep(0.5)
    set_servo_angle(wristServo, 90)
    time.sleep(0.5)
    set_servo_angle(gripperServo, gripper_min)
    # 更新滑桿值
    base_slider.set(90)
    shoulder_slider.set(90)
    elbow_slider.set(90)
    wrist_slider.set(90)
    gripper_slider.set(gripper_min)
    
# 更新底座舵機的角度
def update_base_angle(angle):
    set_servo_angle(baseServo, angle)

# 建立 GUI
root = tk.Tk()
root.title("伺服馬達控制面板")

# 添加肩膀滑桿
shoulder_label = tk.Label(root, text="肩膀舵機")
shoulder_label.pack()
shoulder_slider = tk.Scale(root, from_=shoulder_min, to=shoulder_max, orient=tk.HORIZONTAL, command=update_shoulder_angle)
shoulder_slider.pack()

# 添加手肘滑桿
elbow_label = tk.Label(root, text="手肘舵機")
elbow_label.pack()
elbow_slider = tk.Scale(root, from_=elbow_min, to=elbow_max, orient=tk.HORIZONTAL, command=update_elbow_angle)
elbow_slider.pack()

# 添加手腕滑桿
wrist_label = tk.Label(root, text="手腕舵機")
wrist_label.pack()
wrist_slider = tk.Scale(root, from_=wrist_min, to=wrist_max, orient=tk.HORIZONTAL, command=update_wrist_angle)
wrist_slider.pack()

# 添加夾爪滑桿
gripper_label = tk.Label(root, text="夾爪舵機")
gripper_label.pack()
gripper_slider = tk.Scale(root, from_=gripper_min, to=gripper_max, orient=tk.HORIZONTAL, command=update_gripper_angle)
gripper_slider.pack()

# 添加底座滑桿
base_label = tk.Label(root, text="底座舵機")
base_label.pack()
base_slider = tk.Scale(root, from_=base_min, to=base_max, orient=tk.HORIZONTAL, command=update_base_angle)
base_slider.pack()

default_button = tk.Button(root, text="設置預設角度", command=reset_all_servos)
default_button.pack()

# 開始 GUI 事件循環
root.mainloop()