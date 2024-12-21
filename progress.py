import cv2
import pid
import time
import tof
import pytesseract
import pointinfo
import markerinfo
from PIL import Image
from robomaster import robot

line = []
markers = []
x, y, w, h = 0, 0, 0, 0

class EpRobot:
    def __init__(self, connect = 'ap', kp = 70, ki = 5, kd = 30):
        self.my_robot = robot.Robot()
        self.my_robot.initialize(connect)
        self.my_chassis = self.my_robot.chassis
        self.my_arm = self.my_robot.robotic_arm
        self.my_arm.recenter().wait_for_completed()
        self.my_gripper = self.my_robot.gripper
        self.my_gripper.open()
        self.my_vision = self.my_robot.vision
        self.my_camera = self.my_robot.camera
        self.my_camera.start_video_stream(display = False)
        self.pid_ctrl = pid.PID(kp, ki, kd)
        self.result = 'apple'
        tof.setup()
    def __del__(self):
        self.my_camera.stop_video_stream()
        self.my_robot.close()
        tof.end()

def on_detect_line(line_info):
    line.clear()
    print('line_type', line_info[0])
    for index, item in enumerate(line_info):
        if index == 0:
            continue
        x_, y_, ceta_, c_ = item
        line.append(pointinfo.PointInfo(x_, y_, ceta_, c_))

def on_detect_marker(marker_info):
    global x, y, w, h
    markers.clear()
    for item in marker_info:
        x, y, w, h, info = item
        markers.append(markerinfo.MarkerInfo(x, y, w, h, info))

def recognize(camera):
    result = ''
    while not result:
        img = camera.read_cv2_image(strategy='newest', timeout=0.5)
        pil_image = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        text = pytesseract.image_to_string(pil_image, lang='eng').lower().split('\n\n')
        for string in ('apple', 'pineapple', 'watermelon', 'banana'):
            if string in text:
                result = string
                break
    print('recognize succeed: {}'.format(result))
    return result

def seek(chassis, pos, speed = 5, target = 550, kx = 1, ky = 1, max_speed = 0.2):
    x_speed = 0
    y_speed = 0
    distance = tof.dis()
    if abs(pos - target) > 10:
        y_speed = speed * (pos - target) / kx
    if abs(distance - 20) > 5:
        x_speed = speed * (distance - 30) / ky
    x_speed = x_speed if x_speed <= max_speed else max_speed
    x_speed = x_speed if x_speed >= -max_speed else -max_speed
    y_speed = y_speed if y_speed <= max_speed else max_speed
    y_speed = y_speed if y_speed >= -max_speed else -max_speed
    print(x_speed, y_speed)
    chassis.drive_speed(x=x_speed, y=y_speed, z=0)
    return y_speed, x_speed

def grab(arm, gripper):
    print("catch start")
    gripper.open(power=50)
    time.sleep(3)
    arm.recenter().wait_for_completed()

    #time.sleep(1)
    gripper.pause()
    arm.move(x=0, y=90).wait_for_completed()  # TODO 调参
    arm.move(x=140, y=0).wait_for_completed()  # TODO 调参
    gripper.close(power=50)

    #ime.sleep(1)
    gripper.pause()
    arm.move(x=-140, y=0).wait_for_completed()  # TODO 调参
    arm.recenter().wait_for_completed()
    print("catch end")

def place(arm, gripper):
    print("put start")
    arm.move(x=0, y=90).wait_for_completed()  # TODO 调参
    arm.move(x=140, y=0).wait_for_completed()  # TODO 调参
    gripper.open(power=50)

    #time.sleep(1)
    arm.move(x=-120, y=0).wait_for_completed()  # TODO 调参
    arm.recenter().wait_for_completed()

    #time.sleep(1)
    gripper.pause()
    print("put end")

def move(chassis, camera, vision, pid_ctrl, target_color = 'blue', base_speed = 20, start_angle = 0):
    if start_angle != 0: #turn left
        chassis.move(x = 0, y = 0, z = start_angle, z_speed = 180).wait_for_completed()
    quit_count = 0
    result_line = vision.sub_detect_info(name = 'line', color = target_color, callback = on_detect_line)
    while quit_count <= 10:
        img = camera.read_cv2_image(strategy = 'newest', timeout = 0.5)
        line_1 = line.copy()
        if line_1:
            quit_count = 0
        else:
            quit_count += 1

        min_distance = 1.11
        min_err_x = 0.5
        for index, item in enumerate(line_1):
            cv2.circle(img, item.pt, 3, item.color, -1)
            if item.distance < min_distance:
                min_distance = item.distance
                min_err_x = item._x - 0.5
        #cv2.imshow('Line', img)
        cv2.waitKey(1)
        l_speed = 0
        r_speed = 0
        if min_err_x != 0.5:
            pid_ctrl.set_err(min_err_x)
            dif_speed = pid_ctrl.output
            l_speed = base_speed + dif_speed
            r_speed = base_speed - dif_speed
        chassis.drive_wheels(w2 = l_speed, w3 = l_speed, w1 = r_speed, w4 = r_speed)
        time.sleep(0.1)
    chassis.drive_wheels(0, 0, 0, 0)
    result_line = vision.unsub_detect_info(name = 'line')
    cv2.destroyAllWindows()
    print("move finished")