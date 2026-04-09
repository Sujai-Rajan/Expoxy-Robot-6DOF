#!/usr/bin/env python3

import sys
import math
import time
import queue
import datetime
import random
import traceback
import threading
from xarm import version
from xarm.wrapper import XArmAPI


class RobotMain(object):
    """Robot Main Class"""
    def __init__(self, robot, **kwargs):
        self.alive = True
        self._arm = robot
        self._ignore_exit_state = False
        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500
        self._vars = {}
        self._funcs = {}
        self._robot_init()

    # Robot init
    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)

    # Register error/warn changed callback
    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    # Register state changed callback
    def _state_changed_callback(self, data):
        if not self._ignore_exit_state and data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code, self._arm.connected, self._arm.state, self._arm.error_code, ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def arm(self):
        return self._arm

    @property
    def VARS(self):
        return self._vars

    @property
    def FUNCS(self):
        return self._funcs

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._ignore_exit_state:
                return True
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False

    # Robot Main Run
    def run(self):
        try:
            # PCB_466
            if self._arm.get_cgpio_digital(8)[1]:
                self._tcp_speed = 500
                self._tcp_acc = 5000
                code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_position(*[247.6, 51.6, 104.3, 177.9, -1.5, 85.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                for i in range(int(0.5/ 0.1)):
                    time.sleep(0.1)
                    if not self.is_alive:
                        return
                code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                for i in range(int(2/ 0.1)):
                    time.sleep(0.1)
                    if not self.is_alive:
                        return
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    code = self._arm.set_position(*[411.9, 41.0, 49.8, -156.1, 47.5, 84.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[417.0, 35.2, 46.7, -139.9, 47.5, 84.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    code = self._arm.set_position(*[452.9, 35.2, 46.7, -139.9, 47.5, 84.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[451.0, 43.8, 61.1, -139.9, 47.5, 84.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[462.4, 43.1, 61.1, -139.9, 47.5, 84.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[462.7, 35.8, 46.7, -139.9, 47.5, 84.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    code = self._arm.set_position(*[503.2, 35.8, 47.9, -139.9, 47.5, 84.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[502.1, 31.6, 51.0, -155.4, 38.0, 5.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[502.0, 27.4, 49.1, -155.4, 38.0, 5.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    code = self._arm.set_position(*[502.0, -12.0, 49.5, -155.4, 38.0, 5.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[506.5, -19.6, 56.3, -148.3, 37.0, 85.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[505.7, -23.6, 49.0, -148.3, 37.0, 85.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    code = self._arm.set_position(*[516.7, -23.6, 49.5, -148.3, 37.0, 85.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[518.7, -20.5, 54.0, 168.3, 57.6, 60.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[517.4, -30.8, 54.0, 168.3, 57.6, 60.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[513.1, -31.8, 42.3, 168.3, 57.6, 60.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[513.1, -18.3, 42.3, 168.3, 57.6, 60.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[517.5, -22.6, 51.2, 168.3, 57.6, 60.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[522.5, -38.4, 55.0, -155.3, 48.5, 71.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[522.6, -45.2, 50.6, -155.3, 48.5, 71.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    code = self._arm.set_position(*[561.5, -44.9, 50.7, -155.3, 48.5, 71.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    code = self._arm.set_position(*[570.0, -36.4, 63.2, -132.7, 10.3, 30.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[562.6, -51.5, 49.6, -132.7, 10.3, 30.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    code = self._arm.set_position(*[561.6, -101.6, 51.4, -132.7, 10.3, 30.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[567.9, -96.4, 63.8, -132.7, 10.3, 30.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[567.9, -110.7, 65.8, -132.7, 10.3, 30.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[562.1, -112.4, 52.3, -132.7, 10.3, 30.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    code = self._arm.set_position(*[562.1, -159.1, 51.0, -132.7, 10.3, 30.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[565.0, -197.0, 65.1, -146.5, 44.9, 13.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[505.8, -198.6, 53.8, -146.5, 44.9, 13.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[511.0, -184.9, 50.0, -146.5, 44.9, 13.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    self._tcp_acc = 90
                    self._tcp_speed = 5
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    code = self._arm.set_position(*[556.1, -187.1, 49.2, -146.5, 44.9, 13.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[558.4, -266.4, 83.1, -146.5, 44.9, 13.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[415.9, -260.0, 70.1, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[427.3, -194.0, 49.5, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    code = self._arm.set_position(*[454.1, -194.0, 51.1, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    self._tcp_acc = 4000
                    self._tcp_speed = 300
                    code = self._arm.set_position(*[453.9, -200.5, 61.4, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[462.5, -200.7, 61.4, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[467.1, -194.6, 50.0, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[498.9, -194.6, 51.4, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[502.6, -224.3, 74.9, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[470.3, -220.1, 50.9, -153.4, 30.2, 14.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    self._tcp_acc = 100
                    self._tcp_speed = 10
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    code = self._arm.set_position(*[479.6, -220.1, 52.4, -153.4, 30.2, 14.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[483.6, -222.2, 67.7, -153.4, 30.2, 14.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[467.8, -212.7, 67.6, -162.7, 30.3, 14.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[469.6, -208.5, 49.8, -162.7, 30.3, 14.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[481.9, -208.5, 49.8, -162.7, 30.3, 14.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[483.6, -208.6, 65.0, -179.5, 30.4, 14.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[468.7, -214.7, 48.9, 179.8, 30.4, 14.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    code = self._arm.set_position(*[480.3, -214.7, 48.9, 179.8, 30.4, 14.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_acc = 4000
                    self._tcp_speed = 300
                    code = self._arm.set_position(*[482.6, -213.1, 65.2, -179.5, 30.4, 14.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[486.6, -200.5, 66.7, -167.7, 44.7, 17.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[483.9, -200.5, 52.5, -167.7, 44.7, 17.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[467.5, -200.5, 52.9, -167.7, 44.7, 17.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 5
                    self._tcp_acc = 50
                    code = self._arm.set_position(*[478.5, -202.3, 49.2, -169.0, 44.8, 17.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                # Right, Bottom , Left Complete
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[403.6, -207.5, 124.9, -171.3, 30.3, 9.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[400.6, -202.0, 115.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[411.8, -190.4, 46.0, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[411.8, -182.0, 46.0, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[407.2, -183.7, 55.1, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[407.2, -172.5, 55.1, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[410.3, -169.0, 47.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[410.3, -163.6, 48.2, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[407.2, -165.2, 55.9, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[407.9, -154.4, 55.9, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[409.7, -151.3, 47.5, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[409.7, -146.6, 47.5, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[406.8, -147.8, 57.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[406.8, -135.2, 57.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[409.6, -134.2, 48.0, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[409.6, -129.1, 48.0, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[405.9, -129.5, 56.8, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[405.9, -117.8, 56.8, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[408.4, -114.0, 44.3, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[408.4, -109.2, 44.3, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[403.9, -109.6, 53.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[404.0, -100.5, 53.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[408.6, -97.3, 45.9, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[408.6, -91.8, 45.9, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[404.1, -91.4, 56.6, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[404.7, -64.4, 56.6, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[408.7, -63.1, 48.0, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[408.7, -57.8, 48.0, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[405.4, -55.9, 54.1, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[405.4, -43.6, 54.1, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[408.4, -43.8, 45.9, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[408.4, -38.3, 45.9, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[404.9, -36.4, 53.0, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[404.9, -27.2, 53.0, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[408.1, -25.8, 45.1, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[408.1, -20.6, 45.1, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[404.2, -20.5, 54.8, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[404.2, -8.8, 54.8, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[409.1, -7.6, 45.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[409.1, -2.3, 45.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[403.5, -1.4, 53.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[403.5, 9.7, 53.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[408.9, 9.5, 46.9, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[408.9, 14.5, 46.9, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[403.8, 16.3, 52.3, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[403.8, 28.5, 52.3, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[409.0, 27.7, 46.3, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[409.0, 38.1, 46.3, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                # all exterior done
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[408.2, 42.1, 167.6, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[454.6, -33.0, 128.5, -166.3, -11.1, 65.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[461.8, -33.0, 99.7, -166.3, -11.1, 65.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[454.7, -31.5, 53.6, -160.3, -16.6, 63.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[454.7, -44.5, 52.9, -160.3, -16.6, 63.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[458.6, -44.5, 59.3, -162.8, -16.4, 63.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[454.7, -64.9, 52.8, -160.3, -16.6, 63.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[454.7, -77.1, 51.9, -160.3, -16.6, 63.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[456.7, -77.1, 56.5, -162.4, -16.5, 63.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[454.7, -90.0, 52.0, -160.3, -16.6, 63.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[454.7, -105.5, 52.1, -160.3, -16.6, 63.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[454.7, -104.6, 65.1, -178.2, 10.3, 61.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[454.7, -128.2, 65.1, -178.2, 10.3, 61.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[452.8, -131.0, 51.7, -178.1, 18.5, 61.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[452.8, -116.9, 51.8, -178.1, 18.5, 61.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[462.4, -113.2, 119.2, 157.4, 19.8, 61.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[466.1, -113.2, 56.1, 157.4, 19.8, 61.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[469.3, -131.2, 52.1, 153.4, 22.4, 59.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[469.3, -116.1, 52.1, 153.4, 22.4, 59.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[466.1, -113.4, 64.2, 156.9, 19.6, 61.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[469.3, -100.8, 51.4, 153.4, 22.4, 59.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[469.3, -86.8, 51.5, 153.4, 22.4, 59.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[467.4, -82.8, 65.1, 156.9, 19.6, 61.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[469.3, -71.5, 50.8, 153.4, 22.4, 59.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[469.3, -58.8, 50.1, 153.4, 22.4, 59.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[468.9, -52.4, 74.0, 159.2, 19.5, 60.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[469.0, -145.0, 54.2, 159.2, 21.6, 60.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    self._tcp_speed = 15
                    self._tcp_acc = 300
                    code = self._arm.set_position(*[459.9, -143.2, 52.9, 172.3, 21.1, 60.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=90.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[466.1, -113.2, 116.4, -162.1, -3.9, 1.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[465.4, -27.8, 84.3, -162.1, -3.9, 1.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[459.4, -23.8, 51.6, -162.1, -3.9, 1.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    self._tcp_speed = 15
                    self._tcp_acc = 300
                    code = self._arm.set_position(*[466.1, -24.2, 52.0, -162.1, -3.9, 1.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[468.3, -24.6, 52.9, -162.1, -9.7, 1.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[468.0, -28.1, 64.2, -157.3, -16.7, 1.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[473.5, -30.9, 53.1, -157.3, -16.7, 1.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(0.5/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[473.5, -49.2, 53.2, -157.3, -16.7, 1.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[472.6, -51.7, 130.4, -157.3, -16.7, 1.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[451.2, -26.4, 98.8, -176.6, -6.4, -66.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[434.5, -26.4, 98.8, -176.6, -6.4, -66.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(13/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[436.2, -26.4, 126.0, -176.6, -6.4, -66.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[356.8, -26.4, 126.0, -176.6, -6.4, -66.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[354.5, -149.2, 126.0, -176.6, -6.4, -66.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[451.6, -144.6, 126.0, -176.6, -6.4, -66.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[451.5, -146.4, 102.5, -176.1, -3.9, -66.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[445.6, -146.4, 102.5, -176.1, -3.9, -66.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[437.0, -146.4, 102.5, -176.1, -3.9, -66.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(13/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[454.1, -146.3, 104.1, -176.1, -3.9, -66.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[458.2, -145.7, 120.8, -176.1, -3.9, -66.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[461.5, -86.5, 114.3, -176.1, -3.9, -66.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[496.9, -86.0, 100.2, -176.1, -3.9, -66.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[500.2, -86.0, 100.2, -176.1, -3.9, -66.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(13/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[507.5, -87.9, 118.5, -176.1, -3.9, -66.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[507.4, -151.9, 118.5, -169.9, 20.2, -66.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[505.1, -148.0, 59.1, -169.9, 24.3, -63.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[503.0, -144.5, 52.0, -169.3, 24.3, -63.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[507.9, -144.5, 51.7, -164.4, 24.3, -63.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[516.6, -147.3, 58.5, -159.0, 15.2, -65.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[516.5, -146.0, 51.8, -162.9, 7.3, -63.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    self._tcp_speed = 10
                    self._tcp_acc = 100
                    code = self._arm.set_position(*[516.5, -161.4, 52.8, -162.9, 7.3, -63.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                for i in range(int(1)):
                    if not self.is_alive:
                        break
                    t1 = time.monotonic()
                    self._tcp_speed = 300
                    self._tcp_acc = 4000
                    code = self._arm.set_position(*[514.1, -161.4, 114.6, -162.9, 7.3, -63.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[549.5, -170.1, 114.6, -162.9, 7.3, -63.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[553.8, -166.9, 67.1, -165.6, 0.0, -63.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[547.6, -165.2, 67.1, -168.2, 0.0, -63.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    self._tcp_speed = 3
                    self._tcp_acc = 15
                    code = self._arm.set_position(*[514.3, -165.2, 67.1, -168.2, 0.0, -63.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    self._tcp_acc = 100
                    self._tcp_speed = 10
                    code = self._arm.set_position(*[505.5, -165.2, 54.6, 176.2, 0.0, -63.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    for i in range(int(1/ 0.1)):
                        time.sleep(0.1)
                        if not self.is_alive:
                            return
                    interval = time.monotonic() - t1
                    if interval < 0.01:
                        time.sleep(0.01 - interval)
                self._tcp_speed = 500
                self._tcp_acc = 9000
                code = self._arm.set_position(*[507.8, -163.5, 437.8, -172.2, 0.0, -63.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                self._arm.move_gohome()
            self._tcp_speed = 10
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        finally:
            self.alive = False
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
            self._arm.release_state_changed_callback(self._state_changed_callback)


if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('192.168.1.196', baud_checkset=False)
    time.sleep(0.5)
    robot_main = RobotMain(arm)
    robot_main.run()
