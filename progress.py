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

def setup(ep_robot, kp = 50, ki = 2, kd = 20):
    pid_ctrl = pid.PID(kp, ki, kd)
    tof.setup(ep_robot)
    return pid_ctrl

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
        y_speed = speed * (pos - target) / ky
    if abs(distance - 15) > 3:
        x_speed = speed * (distance - 10) / kx
    x_speed = x_speed if x_speed <= max_speed else max_speed
    x_speed = x_speed if x_speed >= -max_speed else -max_speed
    y_speed = y_speed if y_speed <= max_speed else max_speed
    y_speed = y_speed if y_speed >= -max_speed else -max_speed
    print(x_speed, y_speed)
    chassis.drive_speed(x=x_speed, y=y_speed, z=0)
    return y_speed, x_speed

def grab(arm, gripper, chassis):
    print("catch start")
    gripper.open(power=50)
    # time.sleep(3)
    arm.recenter().wait_for_completed()

    #time.sleep(1)
    #gripper.pause()
    arm.move(x=0, y=90).wait_for_completed()  # TODO 调参
    arm.move(x=100, y=0).wait_for_completed()  # TODO 调参
    gripper.close(power=50)

    time.sleep(1)
    arm.move(x=0, y=20).wait_for_completed()
    chassis.move(x = -0.2, y = 0, xy_speed = 0.5).wait_for_completed()
    arm.move(x=-100, y=0).wait_for_completed()  # TODO 调参
    arm.move(x = 0, y = -100).wait_for_completed()
    #arm.recenter().wait_for_completed()
    print("catch end")

def place(arm, gripper, chassis):
    time.sleep(1)
    print("put start")
    arm.moveto(x=0, y=150).wait_for_completed()  # TODO 调参
    arm.move(x=180, y=0).wait_for_completed()  # TODO 调参
    gripper.open(power=50)
    print("put end")

    time.sleep(1)
    chassis.move(x=-0.2, y=0, xy_speed=0.5).wait_for_completed()
    arm.move(x=-80, y=0).wait_for_completed()  # TODO 调参
    arm.recenter().wait_for_completed()
    print("arm position adjust")

def move(arm, chassis, camera, vision, pid_ctrl, target_color = 'blue', base_speed = 25, start_angle = 0, end_dis = 100):
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
    arm.moveto(x = 0, y = 30).wait_for_completed()
    while tof.dis() > end_dis:
        chassis.drive_speed(x = 0.1, y = 0, z = 0, timeout = 0.1)
        print(tof.dis())
    chassis.drive_speed(x = 0, y = 0, z = 0)
    print("move finished")

if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize('ap')
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False)
    ep_arm = ep_robot.robotic_arm
    ep_arm.recenter().wait_for_completed()
    # grab(ep_arm, ep_robot.gripper, ep_robot.chassis)
    # place(ep_arm, ep_robot.gripper, ep_robot.chassis)

    pid = setup(ep_robot, kp = 62, ki = 5, kd = 43)
    # base_speed p i d
    # 55, 62, 0.1, 55  稳定循迹,但第二个弯会过调 速度提升至50后，转弯后直线段会过调
    # 80, 62, 0.1, 43  较快，出弯进直线调整慢，但是大致轨迹是直线的


    ep_robot.chassis.move(x = -0.2, y = 0, xy_speed = 0.5).wait_for_completed()
    move(arm=ep_arm,chassis=ep_robot.chassis, camera=ep_robot.camera, vision=ep_robot.vision, pid_ctrl=pid, target_color='blue', base_speed=75, start_angle=180)

    ep_robot.chassis.move(x = -0.2, y = 0, xy_speed = 0.5).wait_for_completed()
    move(arm=ep_arm,chassis=ep_robot.chassis, camera=ep_robot.camera, vision=ep_robot.vision, pid_ctrl=pid, target_color='blue', base_speed=80, start_angle=190)

    ep_robot.close()
    