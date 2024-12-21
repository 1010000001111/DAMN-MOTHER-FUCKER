import progress
import cv2
from yolov5 import YOLOv5
import time
from robomaster import robot, led

mod_path = 'Demo/runs/train/exp9_ok/weights/best.pt'
Yolo_v5 = YOLOv5(mod_path)
ep_robot = robot.Robot()
ep_robot.initialize('ap')
ep_camera = ep_robot.camera
ep_camera.start_video_stream(display=False)
result = ''

def detect(cv2img, show_results = False):
    __img__ = cv2.cvtColor(cv2img, cv2.COLOR_BGR2RGB)
    __results__ = Yolo_v5.predict(__img__)
    __names__ = __results__.names
    predictions = __results__.pred[0]
    if 'cuda' in str(predictions.device):
        predictions = predictions.cpu()
    __boxes__ = predictions[:, :4].numpy().tolist()  # x1, x2, y1, y2
    __scores__ = predictions[:, 4].numpy().tolist()
    __categories__ = predictions[:, 5].numpy().tolist()
    if show_results:
        __results__.show()
        print(__results__)
        print(__boxes__)
        print(__scores__)
        print(__categories__)

    return __boxes__, __scores__, __categories__, __names__

def fruit():
    count = 0
    while count < 10:
        img = ep_camera.read_cv2_image(strategy = "newest", timeout = 0.5)
        boxes, scores, categories, names = detect(img, show_results = False)
        flag = False
        for index, item in enumerate(boxes):
            pts = item
            t_class = int(categories[index])
            x1, y1, x2, y2 = list(map(int, pts))
            print(names[t_class], x1, y1, x2, y2)
            if names[t_class] == result:
                tmp_y, tmp_x = progress.seek(chassis = ep_robot.chassis, pos = x1, speed = 5, target = 580, kx = 4000, ky = 3000, max_speed = 0.15)
                if abs(tmp_y) <= 0.05 and abs(tmp_x) <= 0.05:
                    count += 1
                else:
                    count = 0
                flag = True
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 255, 255), 3)
            cv2.putText(img, names[t_class], (x1, y1), cv2.FONT_HERSHEY_COMPLEX_SMALL, fontScale=2, color=(0, 0, 0), thickness=1)
            # if flag:
            #     break
        cv2.imshow("EPCamera", img)
        if not flag:
            progress.seek(chassis = ep_robot.chassis, pos = 0, speed = 5, target = 580, kx = 4000, ky = 3000, max_speed = 0.15)
        cv2.waitKey(1)
    ep_robot.chassis.drive_speed(x = 0, y = 0, z = 0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    ep_arm = ep_robot.robotic_arm
    ep_arm.recenter().wait_for_completed()
    ep_robot.gripper.open()
    pid = progress.setup(ep_robot, kp = 100, ki = 5, kd = 30)
    time.sleep(1)
    result = progress.recognize(camera = ep_robot.camera)
    ep_robot.led.set_led(comp = led.COMP_ALL, r = 133, g = 24, b = 247, effect = led.EFFECT_ON)
    progress.move(chassis = ep_robot.chassis, camera = ep_robot.camera, vision = ep_robot.vision, pid_ctrl = pid, target_color = 'red', base_speed = 30, start_angle = 190)
    fruit()
    progress.grab(arm = ep_arm, gripper = ep_robot.gripper, chassis = ep_robot.chassis)
    progress.move(chassis=ep_robot.chassis, camera=ep_robot.camera, vision=ep_robot.vision, pid_ctrl=pid, target_color='red', base_speed=30, start_angle=190)
    # progress.place(arm = ep_arm, gripper = ep_robot.gripper, chassis = ep_robot.chassis)
    print("damn mother fucker")
    #progress.grab(arm = ep_robot.my_arm, gripper = ep_robot.my_gripper)
    print("catch start")
    # ep_robot.gripper.open(power=50)
    #time.sleep(3)
    #ep_arm.recenter().wait_for_completed()
    print('damn mother fucker')
    ep_robot.chassis.drive_speed(x = 0, y = 0, z = 0)
    print('damn you mother fucker')
    # time.sleep(1)
    # gripper.pause()
    ep_robot.robotic_arm.move(x=0, y=90).wait_for_completed()  # TODO 调参

    print('damn')
    ep_robot.robotic_arm.move(x=140, y=0).wait_for_completed()  # TODO 调参
    print('damn')
    ep_robot.gripper.open(power=50)

    time.sleep(1)
    # gripper.pause()
    ep_robot.robotic_arm.move(x=-140, y=0).wait_for_completed()  # TODO 调参
    ep_robot.robotic_arm.recenter().wait_for_completed()
    print("catch end")
    del ep_robot
    pass