# UR の状態をモニタリングする

from paho.mqtt import client as mqtt

import rtde_control
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from datetime import datetime
import time

import os
import sys
import json
import psutil

import multiprocessing as mp
from multiprocessing import Process, Queue
import multiprocessing.shared_memory

import numpy as np

from dotenv import load_dotenv

load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))
ROBOT_IP = os.getenv("ROBOT_IP", "127.0.0.1")
MQTT_SERVER = os.getenv("MQTT_SERVER", "127.0.0.1")
MQTT_ROBOT_STATE_TOPIC= os.getenv("MQTT_ROBOT_STATE_TOPIC", "ur5e/state")


#robot_ip = "127.0.0.1"
#robot_ip = "10.5.5.102"
vel = 0.5
acc = 0.5
rtde_frequency = 500.0
dt = 1.0/rtde_frequency  # 2ms
flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
ur_cap_port = 50002

lookahead_time = 0.1
gain = 600

# ur_rtde realtime priorities
rt_receive_priority = 90
#rt_control_priority = 85

class UR_MON:
    def __init__(self,verbose=False):
        self.verbose= verbose

    def init_rtde(self):
        self.rtde_r = RTDEReceive(self.robot_ip, rtde_frequency, [], True, False, rt_receive_priority)

    def init_realtime(self):
        os_used = sys.platform
        process = psutil.Process(os.getpid())
        if os_used == "win32":  # Windows (either 32-bit or 64-bit)
            process.nice(psutil.REALTIME_PRIORITY_CLASS)
        elif os_used == "linux":  # linux
            rt_app_priority = 80
            param = os.sched_param(rt_app_priority)
            try:
                os.sched_setscheduler(0, os.SCHED_FIFO, param)
            except OSError:
                print("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
            else:
                print("Process real-time priority set to: %u" % rt_app_priority)


    def on_connect(self,client, userdata, flag, rc,proc):
        print("Connected with result code " + str(rc))  # 接続できた旨表示

# ブローカーが切断したときの処理
    def on_disconnect(self,client, userdata, rc):
        if  rc != 0:
            print("Unexpected disconnection.")

    def connect_mqtt(self):
        self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
# MQTTの接続設定
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.connect(MQTT_SERVER, 1883, 60)
        self.client.loop_start()   # 通信処理開始
#        self.client.loop_forever()   # 通信処理開始

    def monitor_start(self):
        lastStatus = -1
        lastRuntimeState = -1
        last = 0
        while True:
            now = time.time()
            if last == 0:
                last = now
            t_start = self.rtde_r.initPeriod()
#            actual_tcp_pose = self.rtde_r.getActualTCPPose()
            actual_tcp_pose = self.rtde_r.getActualQ()

            # エラー状態を取得したい
            robot_state = self.rtde_r.getRobotStatus()
            if robot_state != lastStatus:
                print("RobotStatus:",robot_state)
                lastStatus = robot_state

            runtime_state = self.rtde_r.getRobotStatus()
            if runtime_state != lastRuntimeState:
                print("RuntimeState:",runtime_state)
                lastRuntimeState = runtime_state

            robot_state = self.rtde_r.getRuntimeState()

            if now-last > 0.3:
                self.client.publish(MQTT_ROBOT_STATE_TOPIC, json.dumps(actual_tcp_pose))
                last = now
            else:
                print("Now-last:",now, last, now-last)
            # ここで SharedMemory を使う！

            if self.verbose:
                n = time.time()
                print(n,actual_tcp_pose)
            self.pose[:len(actual_tcp_pose)] = actual_tcp_pose
#            print("Sleep.. ",t_start)
            self.rtde_r.waitPeriod(t_start)
            time.sleep(0.300)


    def run_proc(self,new_robot_ip=ROBOT_IP):
        self.robot_ip = new_robot_ip
        self.sm = mp.shared_memory.SharedMemory("UR5e")
        self.pose = np.ndarray((12,), dtype=np.dtype("float32"), buffer=self.sm.buf)

        self.init_realtime()
        self.init_rtde()
        self.connect_mqtt()
        try:
            self.monitor_start()
        except KeyboardInterrupt:
            print("Stop! UR monitor")

if __name__ == '__main__':
    verbose = False
    if len(sys.argv)>1:
        if sys.argv[1]=="-v":
            verbose = True
    ur = UR_MON(verbose)
    ur.init_realtime()
    ur.init_rtde()
    ur.connect_mqtt()

    try:
        ur.monitor_start()
    except KeyboardInterrupt:
        print("Monitor Main Stopped")
#        rtde_c.servoStop()
#        rtde_c.stopScript()
