from paho.mqtt import client as mqtt

import rtde_control

from rtde_control import RTDEControlInterface as RTDEControl

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
#robot_ip = "127.0.0.1"
#robot_ip = "10.5.5.102"

rtde_frequency = 500.0
dt = 1.0/rtde_frequency  # 2ms
flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
ur_cap_port = 50002

# Parameters
velocity = 0.4
acceleration = 0.3
dt = 1.0/500  # 2ms
lookahead_time = 0.1
gain = 300

rt_control_priority = 85

class UR_CON:
    def __init__(self):
        self.last = 0
        self.average = np.ndarray((10,),np.dtype("int16"))
        self.average.fill(0)

    def init_rtde(self):
        print("Initializing RTDE control", self.robot_ip)
        try:
            self.rtde_c = RTDEControl(self.robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)
        except Exception as e:
            print("NO RTDE Control!",e)

            exit(1)
            # need to stop other processes!
            print("No..")

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

    def main_loop(self):
        count=0
        print("[CNT]Start Main Loop")
        while self.loop:

            # 現在情報を取得しているかを確認
            if self.pose[0:6].sum() == 0:
                time.sleep(0.3)
                print("[CNT]Wait for monitoring..")
                continue

            if self.pose[6:].sum() == 0:
                time.sleep(0.8)
                print("[CNT]Wait for target..")
                continue 
            
            now = time.time()
            if self.last == 0:
                self.last = now
                print("[CNT]Starting to Control!",self.pose)
                continue
            
            diff = self.pose[:6]-self.pose[6:]
            td = now - self.last
#            self.average[:]=np.roll(self.average,shift=1)
#            self.average[0]=td*10000
#            count +=1
#            if count > 100:
#                print("td",td,self.average.mean(),self.average)
#                count = 0

            ## ここで平滑化したい！
            spd = diff/td
            self.last = now
            joint_q = self.pose[6:].tolist()  #これだと生の値

            t_start = self.rtde_c.initPeriod()
            self.rtde_c.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain)
            self.rtde_c.waitPeriod(t_start)


    def run_proc(self, new_robot_ip=ROBOT_IP):
        self.robot_ip = new_robot_ip

        self.sm = mp.shared_memory.SharedMemory("UR5e")
        self.pose = np.ndarray((12,), dtype=np.dtype("float32"), buffer=self.sm.buf)

        self.loop = True
        self.init_realtime()
        print("[CNT]:start realtime")
        self.init_rtde()
        print("[CNT]:init RTDE")

        try:
            self.main_loop()
        except KeyboardInterrupt:
            print("[CNT]RTDE StopServo/Script")
            self.rtde_c.servoStop()
            self.rtde_c.stopScript()


