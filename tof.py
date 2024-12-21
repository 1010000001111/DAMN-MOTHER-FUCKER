from robomaster import robot

distance = [0, 0, 0, 0]

def sub_data_handler(sub_info):
    global distance
    distance = sub_info
    #print("tof1:{0}  tof2:{1}  tof3:{2}  tof4:{3}".format(distance[0], distance[1], distance[2], distance[3]))

def setup(ep_robot):
    global ep_sensor
    # ep_robot = robot.Robot()
    # ep_robot.initialize('ap')
    ep_sensor = ep_robot.sensor
    ep_sensor.sub_distance(freq = 5, callback = sub_data_handler)

def dis():
    return distance[0]

def end():
    ep_sensor.unsub_distance()