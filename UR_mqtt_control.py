
import json
from paho.mqtt import client as mqtt
import multiprocessing as mp
import multiprocessing.shared_memory
import socket

from multiprocessing import Process, Queue

import sys
import os
from datetime import datetime
import math
import numpy as np
import time

## ここでUUID を使いたい
import uuid

import UR_monitor
import UR_control

from dotenv import load_dotenv
import ipget


load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))

MQTT_SERVER = os.getenv("MQTT_SERVER", "127.0.0.1")
MQTT_CTRL_TOPIC = os.getenv("MQTT_CTRL_TOPIC", "urdemo/demo_ctl")
ROBOT_UUID = os.getenv("ROBOT_UUID","no-uuid")
ROBOT_MODEL = os.getenv("ROBOT_MODEL","no-model")
MQTT_MANAGE_TOPIC = os.getenv("MQTT_MANAGE_TOPIC", "/dev")
MQTT_MANAGE_RCV_TOPIC = os.getenv("MQTT_MANAGE_RCV_TOPIC", "/devctl")+"/"+ROBOT_UUID
MQTT_VACUUM_TOPIC =os.getenv("MQTT_VACUUM_TOPIC", "/vacuum")

#
#  RTDE : UR-5e を動かすためのリアルタイム通信
#
#　MQTT ： Joint、もしくは ツール位置を受信する
#
#　　

"""
   メンテナンスフリーにするためには、 RTDE側の状況を見守る必要がある。

  安定した実行のため、マルチプロセスを使う
    1. VRゴーグルからMQTT を受信するプロセス（目標値） 2 との間でQueue を利用
    2. RTDE経由でURを制御するプロセス（制御値）2との間で Queue, 3 との間で SharedMemory
    3. UR をモニタリングするプロセス
    　　→常に最新情報を提示　（SharedMemoryを利用）


"""

joints=['j1','j2','j3','j4','j5','j6']

def get_ip_list():
    ll = ipget.ipget()
    flag = False
    ips = []
    for p in ll.list:
        if flag:
            flag=False
            if p == "127.0.0.1/8":
                continue
            ips.append(p)
        if p == "inet":
            flag = True
    return ips

class UR_MQTT:
    def __init__(self):
        self.start = -1
 #       self.log = open(fname,"w")

    def on_connect(self,client, userdata, flag, rc):
        print("MQTT:Connected with result code " + str(rc), "subscribe ctrl", MQTT_CTRL_TOPIC)  # 接続できた旨表示
        self.client.subscribe(MQTT_CTRL_TOPIC) #　connected -> subscribe

        # ここで、MyID Register すべき
        my_info = {
            "robot_model": ROBOT_MODEL,
            "IP": get_ip_list(),
            "ID":ROBOT_UUID 
        }
        self.client.publish(MQTT_MANAGE_TOPIC+"/register", json.dumps(my_info))
        print("Publish",json.dumps(my_info))
        self.client.publish(MQTT_MANAGE_TOPIC+"/"+ROBOT_UUID, json.dumps({"date": str(datetime.today())}))

        self.client.subscribe(MQTT_MANAGE_RCV_TOPIC) #　connected -> subscribe
        self.client.subscribe(MQTT_VACUUM_TOPIC) #　connected -> subscribe
        



# ブローカーが切断したときの処理
    def on_disconnect(self,client, userdata, rc):
        if  rc != 0:
            print("Unexpected disconnection.")

    def on_message(self,client, userdata, msg):
#        print("Message",msg.payload)
        if msg.topic == MQTT_CTRL_TOPIC:
            js = json.loads(msg.payload)
            try:
                if js['a']== True:
                    self.start = 0
                    print("Start controlling!")
                elif self.start < 0:
                    print("Waiting...for A button")
                    return

                if self.start > 100 and js['a']==True:
                    self.start = -1
                    return
            except KeyError:
                print("No abutton")
                print(js)

            self.start +=1

            rot =[js[x]  for x in joints]    
            rot2 = [rot[0]+90,-rot[1]-90,-rot[2],-rot[3]-90,rot[4],rot[5]]

# 時刻
#        ctime = datetime.now().strftime("%Y/%m/%d %H:%M:%S.%f")
#        self.log.write(json.dumps({"time":ctime, "recv":rot, "real":real_joints})+"\n")

            joint_q = [math.radians(x) for x in rot2]
        # このjoint 情報も Shared Memoryに保存すべし！
            self.pose[6:] = joint_q 
        # Target 情報を保存するだけ
        else:
            print("not subscribe msg",msg.topic)


    def connect_mqtt(self):

        self.client = mqtt.Client()  
# MQTTの接続設定
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.on_message = self.on_message         # メッセージ到着時のコールバック
        self.client.connect(MQTT_SERVER, 1883, 60)
#  client.loop_start()   # 通信処理開始
        self.client.loop_forever()   # 通信処理開始

    def run_proc(self):
        self.sm = mp.shared_memory.SharedMemory("UR5e")
        self.pose = np.ndarray((12,), dtype=np.dtype("float32"), buffer=self.sm.buf)

        self.connect_mqtt()

class ProcessManager:
    def __init__(self):
        mp.set_start_method('spawn')
        sz = 32* np.dtype('float').itemsize
        self.sm = mp.shared_memory.SharedMemory(create=True,size = sz, name='UR5e')
#        self.sm = mp.shared_memory.SharedMemory(size=sz, name='UR5e')
        self.ar = np.ndarray((12,), dtype=np.dtype("float32"), buffer=self.sm.buf) # 共有メモリ上の Array

    def startRecvMQTT(self):
        self.recv = UR_MQTT()
        self.recvP = Process(target=self.recv.run_proc, args=(),name="MQTT-recv")
        self.recvP.start()

    def startMonitor(self):
        self.mon = UR_monitor.UR_MON()
        self.monP = Process(target=self.mon.run_proc, args=(),name="UR-monitor")
        self.monP.start()

    def startControl(self):
        self.ctrl = UR_control.UR_CON()
        self.ctrlP = Process(target=self.ctrl.run_proc, args=(),name="UR-control")
        self.ctrlP.start()

    def checkSM(self):
        while True:
            diff = self.ar[6:]-self.ar[:6]
            diff *=1000
            diff = diff.astype('int')
            print(self.ar[:6],self.ar[6:])
            print(diff)
            time.sleep(2)
    
                                

if __name__ == '__main__':
#        
    pm = ProcessManager()
    try:
        print("Monitor!")
        pm.startMonitor()
        print("MQTT!")
        pm.startRecvMQTT()
        print("Control")
        pm.startControl()
        print("Check!")
        pm.checkSM()
    except KeyboardInterrupt:
        print("Stop!")
        #rtde_c.servoStop()
        #rtde_c.stopScript()
