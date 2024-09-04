import rtde_control
import json
from paho.mqtt import client as mqtt
import sys
import os
from datetime import datetime
import math


rtde_c = rtde_control.RTDEControlInterface("127.0.0.1")

# Parameters
velocity = 0.5
acceleration = 0.5
dt = 1.0/500  # 2ms
lookahead_time = 0.1
gain = 300
#joint_q = [-1.54, -1.83, -2.28, -0.59, 1.60, 0.023]

joints=['j1','j2','j3','j4','j5','j6']

class UR_MQTT:
    def __init__(self,fname):
        self.start = -1
        self.log = open(fname,"w")

    def on_connect(self,client, userdata, flag, rc):
        print("Connected with result code " + str(rc))  # 接続できた旨表示
#        self.client.subscribe("lss4dof/state") #　connected -> subscribe
        self.client.subscribe("webxr2/joint") #　connected -> subscribe

# ブローカーが切断したときの処理
    def on_disconnect(self,client, userdata, rc):
        if  rc != 0:
            print("Unexpected disconnection.")

    def on_message(self,client, userdata, msg):
#        print("Message",msg.payload)
        js = json.loads(msg.payload)
        try:
            if js['a']== True:
                 self.start = 0
            elif self.start < 0:
                 print("Waiting...")
                 return

            if self.start > 100 and js['a']==True:
                self.client.disconnect()
                self.log.close()
                return
        except KeyError:
            print("No abutton")
            print(js)

        self.start +=1
        # 一定時間動作したら、abutton で停止
#        print("rot:",js)
#        rot = [int(float(x)*10)  for x in js['rotate']]        
##        rot = [math.radians(x)  for x in js['rotate']]
        rot =[js[x]  for x in joints]    
        rot2 = [rot[0],-rot[1]-90,-rot[2],-rot[3]-90,rot[4],rot[5]]
#        rot = [math.radians(x)  for x in js['rotate']]        
        print("rot :",rot)
        print("rot2:",rot2)

        real_joints = []

# 時刻
        ctime = datetime.now().strftime("%Y/%m/%d %H:%M:%S.%f")
        self.log.write(json.dumps({"time":ctime, "recv":rot, "real":real_joints})+"\n")

        t_start = rtde_c.initPeriod()
        joint_q = [math.radians(x) for x in rot2]
        rtde_c.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain)
        rtde_c.waitPeriod(t_start)


    def connect_mqtt(self):
        self.client = mqtt.Client()  
# MQTTの接続設定
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.on_message = self.on_message         # メッセージ到着時のコールバック
        self.client.connect("192.168.207.22", 1883, 60)
#  client.loop_start()   # 通信処理開始
        self.client.loop_forever()   # 通信処理開始



fname = sys.argv[1]
if fname == "":
    fname = "lss4dof2.log"
mq = UR_MQTT(fname)

mq.connect_mqtt()

# Move to initial joint position with a regular moveJ
#rtde_c.moveJ(joint_q)

# Execute 500Hz control loop for 2 seconds, each cycle is 2ms

rtde_c.servoStop()
rtde_c.stopScript()

