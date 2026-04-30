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
            # PCB_466_final_edit
            for i in range(int(1)):
                if not self.is_alive:
                    break
                t1 = time.monotonic()
                if self._arm.get_cgpio_digital(8)[1]:
                    self._tcp_speed = 500
                    self._tcp_acc = 5000
                    code = self._arm.set_position(*[238.5, 145.5, 99.8, -176.9, 0.3, 88.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
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
                        code = self._arm.set_position(*[411.9, 41.0, 56.7, -156.1, 47.5, 84.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[417.0, 35.2, 46.7, -139.9, 47.5, 84.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        self._tcp_speed = 20
                        self._tcp_acc = 4000
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_position(*[446.5, 35.2, 46.7, -139.9, 47.5, 84.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[451.0, 40.6, 61.1, -139.9, 47.5, 84.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[462.4, 41.1, 61.1, -139.9, 47.5, 84.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        self._tcp_speed = 20
                        self._tcp_acc = 4000
                        code = self._arm.set_position(*[462.7, 35.8, 46.7, -139.9, 47.5, 84.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        code = self._arm.set_position(*[493.5, 35.8, 47.9, -139.9, 47.5, 84.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_position(*[493.5, 40.7, 52.8, -139.9, 47.5, 84.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[501.8, 35.2, 52.8, -139.9, 47.5, 84.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        self._tcp_speed = 300
                        self._tcp_acc = 4000
                        code = self._arm.set_position(*[501.7, 32.2, 53.6, -155.4, 38.0, 5.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        self._tcp_speed = 30
                        self._tcp_acc = 6000
                        code = self._arm.set_position(*[498.8, 25.7, 50.4, -155.4, 38.0, 5.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        code = self._arm.set_position(*[500.0, -8.7, 49.5, -155.4, 38.0, 5.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        # cap2
                        interval = time.monotonic() - t1
                        if interval < 0.01:
                            time.sleep(0.01 - interval)
                    for i in range(int(1)):
                        if not self.is_alive:
                            break
                        t1 = time.monotonic()
                        self._tcp_speed = 300
                        self._tcp_acc = 4000
                        code = self._arm.set_position(*[502.1, -16.8, 56.3, -148.3, 37.0, 85.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[505.7, -23.6, 49.0, -148.3, 37.0, 85.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(1/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 20
                        self._tcp_acc = 4000
                        code = self._arm.set_position(*[505.7, -17.7, 55.4, -148.3, 37.0, 85.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 300
                        self._tcp_acc = 4000
                        code = self._arm.set_position(*[508.6, -20.3, 54.0, 168.3, 57.6, 60.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[515.0, -22.7, 54.0, 168.3, 57.6, 60.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[505.5, -34.6, 42.3, 168.3, 57.6, 60.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 20
                        self._tcp_acc = 4000
                        code = self._arm.set_position(*[505.5, -25.0, 42.3, 168.3, 57.6, 60.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        # cap3
                        interval = time.monotonic() - t1
                        if interval < 0.01:
                            time.sleep(0.01 - interval)
                    for i in range(int(1)):
                        if not self.is_alive:
                            break
                        t1 = time.monotonic()
                        self._tcp_speed = 300
                        self._tcp_acc = 4000
                        code = self._arm.set_position(*[507.9, -24.8, 53.7, 168.3, 57.6, 60.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[510.0, -39.0, 53.7, 168.3, 57.6, 60.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[512.7, -36.2, 56.1, -135.0, 48.5, 71.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[520.9, -42.1, 48.5, -134.9, 48.5, 71.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        self._tcp_speed = 20
                        self._tcp_acc = 4000
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        code = self._arm.set_position(*[556.1, -42.1, 48.5, -134.9, 48.5, 71.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        self._tcp_speed = 300
                        self._tcp_acc = 4000
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_position(*[570.0, -36.4, 63.2, -132.7, 10.3, 30.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[562.6, -54.4, 49.6, -132.7, 10.3, 30.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        self._tcp_speed = 20
                        self._tcp_acc = 4000
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        code = self._arm.set_position(*[561.6, -96.8, 51.4, -132.7, 10.3, 30.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        # cap 4 
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
                        self._tcp_speed = 20
                        self._tcp_acc = 4000
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
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
                        # cap5
                        interval = time.monotonic() - t1
                        if interval < 0.01:
                            time.sleep(0.01 - interval)
                    for i in range(int(1)):
                        if not self.is_alive:
                            break
                        t1 = time.monotonic()
                        # 4x parts block
                        self._tcp_speed = 300
                        self._tcp_acc = 4000
                        code = self._arm.set_position(*[565.0, -197.0, 65.1, -146.5, 44.9, 13.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[507.3, -198.6, 53.8, -146.5, 44.9, 13.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[511.0, -184.9, 50.0, -146.5, 44.9, 13.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        self._tcp_speed = 20
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        code = self._arm.set_position(*[554.1, -187.1, 49.2, -146.5, 44.9, 13.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        self._tcp_acc = 1000
                        code = self._arm.set_position(*[554.1, -189.7, 76.0, -146.5, 44.9, 13.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[520.5, -150.8, 74.3, -152.9, 25.8, 21.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[527.9, -150.8, 61.2, -152.9, 25.8, 21.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 5
                        code = self._arm.set_position(*[550.5, -150.8, 61.2, -152.9, 25.8, 21.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 30
                        code = self._arm.set_position(*[563.7, -150.8, 52.0, -152.9, 25.8, 21.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[551.4, -154.6, 52.0, -159.1, 25.8, 21.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(1/ 0.1)):
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
                        self._tcp_speed = 400
                        self._tcp_acc = 1000
                        code = self._arm.set_position(*[563.6, -159.0, 80.7, -152.9, 25.8, 21.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[558.4, -266.4, 83.1, -146.5, 44.9, 13.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[415.9, -260.0, 70.1, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[403.8, -197.8, 67.4, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[417.3, -195.5, 54.8, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        self._tcp_speed = 20
                        self._tcp_acc = 4000
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        code = self._arm.set_position(*[431.4, -194.5, 48.1, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[428.6, -194.5, 48.1, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[452.1, -194.0, 48.4, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_acc = 4000
                        self._tcp_speed = 300
                        code = self._arm.set_position(*[453.9, -200.5, 61.4, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[462.5, -200.7, 61.4, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[467.1, -194.6, 49.9, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 20
                        self._tcp_acc = 4000
                        code = self._arm.set_position(*[498.9, -194.6, 50.5, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        # cap 6 and 7
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
                        code = self._arm.set_position(*[470.5, -212.4, 67.1, -157.5, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[473.3, -215.2, 49.2, -159.5, 30.2, 14.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        self._tcp_acc = 4000
                        self._tcp_speed = 10
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        code = self._arm.set_position(*[482.5, -209.4, 49.2, -159.5, 30.2, 14.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 300
                        self._tcp_acc = 4000
                        code = self._arm.set_position(*[478.0, -213.3, 65.6, -160.7, 30.2, 14.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[477.9, -226.7, 70.0, -150.3, 30.1, 14.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[472.3, -218.6, 51.5, -150.3, 30.2, 14.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 10
                        self._tcp_acc = 4000
                        code = self._arm.set_position(*[481.5, -218.6, 51.5, -150.3, 30.2, 14.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 300
                        self._tcp_acc = 4000
                        code = self._arm.set_position(*[483.8, -228.5, 69.3, -150.3, 30.2, 14.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[468.5, -239.8, 66.2, 141.9, -10.5, -83.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[468.5, -216.4, 66.5, 141.9, -10.5, -83.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[475.4, -223.0, 44.8, 141.9, -12.4, -83.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        self._tcp_speed = 10
                        self._tcp_acc = 4000
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        code = self._arm.set_position(*[482.4, -223.6, 46.4, 141.9, -12.4, -83.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_acc = 4000
                        self._tcp_speed = 300
                        code = self._arm.set_position(*[480.3, -219.8, 67.2, 141.9, -12.4, -83.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[467.2, -207.7, 65.8, -179.7, 6.8, -93.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[475.2, -204.5, 62.5, -179.7, 6.8, -93.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[475.0, -198.0, 53.4, -179.7, -0.5, -80.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(2/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_acc = 4000
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        code = self._arm.set_position(*[489.2, -197.9, 69.4, 171.1, 1.7, -83.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
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
                        code = self._arm.set_position(*[489.2, -248.9, 69.4, 171.1, 1.7, -83.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[403.6, -241.4, 97.8, -171.3, 30.3, 9.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[376.7, -224.3, 88.2, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[411.8, -190.4, 46.0, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 12
                        self._tcp_acc = 4000
                        code = self._arm.set_position(*[411.8, -182.0, 46.0, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[408.8, -183.6, 55.1, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[408.2, -172.5, 55.1, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[410.5, -172.0, 47.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 12
                        self._tcp_acc = 4000
                        code = self._arm.set_position(*[410.3, -164.8, 47.0, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[409.1, -164.4, 55.9, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[409.2, -154.4, 55.9, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[409.7, -151.3, 47.5, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 12
                        code = self._arm.set_position(*[409.7, -146.8, 46.6, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[409.1, -147.8, 57.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[409.0, -135.2, 55.9, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[409.6, -131.9, 45.1, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 12
                        code = self._arm.set_position(*[409.6, -128.2, 45.9, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[409.3, -129.5, 56.8, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[409.3, -117.8, 56.8, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[410.6, -114.1, 44.3, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 12
                        code = self._arm.set_position(*[410.2, -108.8, 44.0, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[409.3, -109.6, 53.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[409.8, -100.5, 53.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[411.3, -97.0, 45.9, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 12
                        code = self._arm.set_position(*[410.8, -91.5, 45.2, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[409.5, -91.4, 56.6, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[409.9, -64.4, 56.6, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[410.7, -60.2, 44.8, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 12
                        code = self._arm.set_position(*[411.1, -56.0, 44.2, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[409.5, -55.9, 54.1, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[409.2, -43.6, 54.1, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[412.2, -42.2, 45.9, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 12
                        code = self._arm.set_position(*[412.2, -39.4, 45.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[409.5, -36.4, 53.0, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[409.5, -27.2, 53.0, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[412.9, -24.0, 45.1, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 12
                        code = self._arm.set_position(*[413.0, -19.6, 45.1, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[409.5, -20.5, 54.8, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[409.4, -8.8, 54.8, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[410.6, -7.6, 45.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 12
                        code = self._arm.set_position(*[411.6, -2.7, 45.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[410.0, -1.4, 53.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[409.7, 9.7, 53.7, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[413.7, 11.9, 45.3, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 12
                        code = self._arm.set_position(*[412.7, 15.8, 45.3, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[409.5, 16.3, 52.3, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[413.5, 28.5, 52.3, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[411.8, 29.1, 45.1, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 12
                        code = self._arm.set_position(*[411.5, 37.2, 46.3, 129.7, 20.3, 40.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[454.7, -27.4, 52.6, -160.3, -16.6, 63.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 15
                        code = self._arm.set_position(*[454.7, -44.1, 50.6, -160.3, -16.6, 63.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[458.6, -44.5, 59.3, -162.8, -16.4, 63.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[454.7, -59.7, 51.5, -160.3, -16.6, 63.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 15
                        code = self._arm.set_position(*[454.7, -77.1, 50.3, -160.3, -16.6, 63.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[456.7, -77.1, 56.5, -162.4, -16.5, 63.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[454.7, -90.0, 51.3, -160.3, -16.6, 63.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 15
                        code = self._arm.set_position(*[454.7, -105.5, 51.8, -160.3, -16.6, 63.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[454.7, -104.6, 65.1, -178.2, 10.3, 61.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[454.7, -128.2, 65.1, -178.2, 10.3, 61.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[452.8, -131.0, 49.0, -178.1, 18.5, 61.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 15
                        code = self._arm.set_position(*[452.8, -112.5, 49.0, -178.1, 18.5, 61.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[462.4, -113.2, 119.2, 157.4, 19.8, 61.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[466.1, -113.2, 56.1, 157.4, 19.8, 61.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[469.3, -131.2, 48.9, 153.4, 22.4, 59.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 15
                        code = self._arm.set_position(*[469.3, -112.6, 48.9, 153.4, 22.4, 59.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[466.1, -113.4, 64.2, 156.9, 19.6, 61.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[469.3, -99.8, 47.3, 153.4, 22.4, 59.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 15
                        code = self._arm.set_position(*[469.3, -82.0, 47.2, 153.4, 22.4, 59.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[467.4, -82.8, 65.1, 156.9, 19.6, 61.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[469.3, -70.3, 46.4, 153.4, 22.4, 59.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 15
                        code = self._arm.set_position(*[469.3, -58.8, 46.4, 153.4, 22.4, 59.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[468.9, -52.4, 74.0, 159.2, 19.5, 60.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[469.0, -145.0, 53.3, 159.2, 21.6, 60.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 22
                        code = self._arm.set_position(*[459.9, -143.2, 52.9, 172.3, 21.1, 60.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=90.0, wait=True)
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
                        code = self._arm.set_position(*[466.1, -113.2, 116.4, -162.1, -3.9, 1.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[465.4, -27.8, 84.3, -162.1, -3.9, 1.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[459.4, -20.1, 51.8, -162.1, -3.9, 1.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 22
                        code = self._arm.set_position(*[466.1, -18.6, 52.9, -162.1, -3.9, 1.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[469.7, -19.3, 54.3, -162.1, -9.7, 1.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        self._tcp_speed = 15
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
                        code = self._arm.set_position(*[464.1, -26.4, 98.8, -176.6, -6.4, -66.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[459.0, -26.2, 98.8, -176.6, -6.4, -66.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[438.5, -26.0, 98.8, -176.6, -6.4, -66.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(15/ 0.1)):
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
                        code = self._arm.set_position(*[460.2, -146.4, 102.5, -176.1, -3.9, -66.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[454.2, -146.4, 102.5, -176.1, -3.9, -66.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[437.0, -146.4, 102.5, -176.1, -3.9, -66.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(15/ 0.1)):
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
                        code = self._arm.set_position(*[463.2, -146.3, 104.1, -176.1, -3.9, -66.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[458.2, -145.7, 120.8, -176.1, -3.9, -66.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[461.5, -86.5, 114.3, -176.1, -3.9, -66.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[471.7, -86.5, 98.7, -176.1, -3.9, -66.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[496.9, -86.0, 100.2, -176.1, -3.9, -66.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(15/ 0.1)):
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
                        code = self._arm.set_position(*[503.0, -145.1, 52.4, -169.3, 24.3, -63.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[513.0, -144.8, 52.9, -164.4, 24.3, -63.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[518.1, -146.4, 52.2, -162.9, 7.3, -63.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 20
                        self._tcp_acc = 100
                        code = self._arm.set_position(*[519.9, -159.6, 52.8, -162.9, 7.3, -63.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[509.3, -151.7, 112.2, -162.9, 7.3, -63.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[516.3, -153.6, 87.8, 164.0, -7.6, -0.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[513.4, -159.8, 52.1, 164.0, -5.1, -0.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(1/ 0.1)):
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
                    self._tcp_speed = 500
                    self._tcp_acc = 9000
                    code = self._arm.set_position(*[507.8, -163.5, 170.2, -172.2, 0.0, -63.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                    if not self._check_code(code, 'set_position'):
                        return
                    self._arm.arm.wait_move()
                    current_angle = self._arm.angles
                    angle_5 = -(current_angle[1] - current_angle[2])
                    angles = [*current_angle[:3],0,angle_5,current_angle[5]]
                    code = self._arm.set_servo_angle(angle=angles)
                    if not self._check_code(code, 'set_end_level'):
                        return
                    self._tcp_speed = 500
                    self._tcp_acc = 9000
                    code = self._arm.set_position(*[247.6, 51.6, 104.3, 177.9, -1.5, 85.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                interval = time.monotonic() - t1
                if interval < 0.01:
                    time.sleep(0.01 - interval)
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        finally:
            self.alive = False
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
            self._arm.release_state_changed_callback(self._state_changed_callback)


if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('10.40.17.196', baud_checkset=False)
    time.sleep(0.5)
    robot_main = RobotMain(arm)
    robot_main.run()
