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
        self._vars = {'flag_d': 0, 'flag_i': 0, 'flag_m': 0, 'flag_e': 0}
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
            self._vars['flag_e'] = True
            self._vars['flag_i'] = True
            self._vars['flag_d'] = True
            self._vars['flag_m'] = True
            self._tcp_speed = 500
            self._tcp_acc = 5000
            code = self._arm.set_position(*[238.5, 145.5, 99.8, -176.9, 0.3, 88.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            if self._vars.get('flag_e', 0):
                if True:
                    # C14
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[361.8, -23.9, 66.8, 151.8, 42.1, 87.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[409.5, -58.2, 45.7, 151.8, 42.1, 87.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[409.4, -23.4, 45.7, 151.8, 42.1, 87.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C36
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[404.8, -26.3, 51.3, 151.8, 42.1, 87.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[355.6, -24.1, 78.0, 151.8, 42.1, 87.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[358.3, 164.6, 78.0, -138.9, 33.9, 87.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[409.9, 153.3, 56.0, -138.9, 33.9, 87.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[409.9, 106.5, 56.0, -138.9, 33.9, 87.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[414.9, 103.9, 46.2, -138.9, 33.9, 87.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[451.5, 104.0, 46.2, -138.9, 33.9, 87.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C38
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[449.5, 106.6, 52.5, -138.9, 33.9, 87.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[460.0, 106.9, 51.3, -138.9, 33.9, 87.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[463.5, 104.4, 45.2, -138.9, 33.9, 87.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[500.9, 103.5, 45.2, -138.9, 33.9, 87.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C15
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[495.9, 105.5, 51.9, -138.9, 33.9, 87.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[533.4, 105.5, 51.9, -138.9, 33.9, 87.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[540.8, 21.6, 57.8, -174.3, 49.1, 73.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[499.9, 21.6, 56.1, -174.3, 49.1, 73.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[500.1, -11.6, 53.7, -174.3, 49.1, 73.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[495.5, -4.9, 45.1, -174.3, 49.1, 73.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[495.5, 2.6, 46.4, -174.3, 49.1, 73.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C15
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[500.0, 2.6, 55.5, -174.3, 49.1, 73.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[500.0, 18.3, 55.5, -174.3, 49.1, 73.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[494.3, 21.9, 46.2, -174.3, 49.1, 73.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[494.7, 43.3, 45.7, -174.3, 49.1, 73.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C17
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[500.0, 41.3, 54.0, -174.3, 49.1, 73.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[500.0, 50.8, 54.0, -174.3, 49.1, 73.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[494.6, 55.2, 45.6, -174.3, 49.1, 73.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[494.3, 103.8, 45.0, -174.3, 49.1, 73.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C39
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[500.0, 100.6, 55.0, -174.3, 49.1, 73.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[530.4, 100.7, 54.9, -141.0, 34.1, 70.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[539.2, 18.0, 58.5, -141.0, 34.1, 70.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[500.9, -12.3, 58.5, -141.0, 34.1, 70.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[505.0, -18.7, 48.4, -141.0, 34.1, 70.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[512.8, -18.7, 49.4, -141.0, 34.1, 70.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C39
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[511.0, -14.0, 58.8, -141.0, 34.1, 70.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[519.6, -14.0, 58.8, -141.0, 34.1, 70.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[523.1, -18.8, 48.4, -141.0, 34.1, 70.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[559.8, -18.8, 48.6, -141.0, 34.1, 70.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C8
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[558.0, -14.6, 57.4, -141.0, 34.1, 70.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[554.3, 20.2, 57.5, 174.0, 52.3, 65.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[620.3, 20.2, 57.5, 174.0, 52.3, 65.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[620.3, -128.0, 57.5, 174.0, 52.3, 65.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[559.7, -128.8, 54.9, 174.0, 52.3, 65.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[555.6, -128.7, 47.5, 174.0, 52.3, 65.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[555.6, -77.9, 47.5, 174.0, 52.3, 65.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C39
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[559.7, -81.1, 58.6, 174.0, 52.3, 65.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[560.1, -69.2, 58.6, 174.0, 52.3, 65.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[553.1, -66.1, 46.9, 174.0, 52.3, 65.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[553.1, -19.0, 46.9, 174.0, 52.3, 65.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C23
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[559.8, -19.0, 54.9, 174.0, 52.3, 65.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[560.7, 21.0, 67.0, 174.0, 52.3, 65.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[523.3, 174.9, 58.1, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[270.8, 151.8, 103.2, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[270.8, -285.8, 95.1, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[412.8, -281.4, 72.6, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[412.8, -195.5, 58.8, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[418.4, -185.4, 49.0, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[454.0, -185.6, 49.0, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C21
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[449.1, -195.0, 58.8, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[458.7, -195.0, 58.8, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[464.1, -185.5, 49.4, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[500.7, -185.8, 49.4, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C8
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[495.5, -195.0, 57.8, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[507.3, -194.6, 72.4, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[503.2, -135.0, 60.3, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[505.4, -126.2, 52.2, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[513.2, -126.2, 52.2, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C8
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[510.3, -135.0, 60.0, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[522.3, -135.0, 60.0, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[525.6, -126.1, 49.6, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[562.7, -125.5, 49.6, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C9
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[559.1, -135.0, 58.6, -127.8, 1.6, 58.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[657.4, -144.7, 58.4, -135.0, -4.3, 36.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[501.7, -137.9, 58.4, -135.0, -4.3, 36.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[500.5, -143.4, 50.3, -135.0, -4.3, 36.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[500.5, -156.2, 50.3, -135.0, -4.3, 36.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C9
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[500.1, -155.1, 57.3, -135.0, -4.3, 36.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[500.1, -168.2, 57.3, -135.0, -4.3, 36.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[498.9, -172.2, 49.8, -135.0, -4.3, 36.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 20
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_position(*[498.9, -191.7, 49.8, -135.0, -4.3, 36.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
            if self._vars.get('flag_i', 0):
                if True:
                    # C14 I
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[500.8, -187.7, 59.2, -135.0, -4.3, 36.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[503.4, -269.0, 59.2, -135.0, -4.3, 36.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[503.4, -281.6, 139.6, -174.3, -5.8, 36.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[458.5, -104.6, 139.6, -174.3, -5.8, 36.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[454.1, -19.4, 55.8, -155.7, -7.1, 34.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[450.9, -21.5, 50.1, -155.7, -7.1, 34.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 20
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_position(*[450.9, -28.6, 50.1, -155.7, -7.1, 34.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C14 I
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[451.5, -26.5, 55.8, -155.7, -7.1, 34.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[451.5, -41.3, 55.8, -155.7, -7.1, 34.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[450.9, -42.4, 49.9, -155.7, -7.1, 34.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 20
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_position(*[450.9, -74.8, 49.9, -155.7, -7.1, 34.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C12 I
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[451.5, -71.5, 55.8, -155.7, -7.1, 34.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[451.5, -84.4, 57.0, -161.8, -1.2, 34.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[450.9, -86.7, 49.6, -161.8, -1.2, 34.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[450.9, -117.4, 49.6, -161.8, -1.2, 34.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C13 I
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[451.5, -114.6, 54.7, -161.8, -1.2, 34.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[453.8, -110.0, 53.7, -164.0, 15.9, -20.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[466.0, -90.4, 54.4, -164.0, -7.5, -20.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[466.0, -18.5, 54.4, -164.0, -7.5, -20.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[470.0, -22.3, 50.4, -163.9, -10.3, -20.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 20
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_position(*[470.0, -27.8, 50.5, -163.9, -10.3, -20.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C13 I
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[466.5, -27.1, 53.1, -164.2, -9.6, -20.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[466.5, -39.1, 53.1, -164.2, -9.6, -20.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[470.0, -40.6, 50.5, -163.9, -10.3, -20.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 20
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_position(*[470.0, -72.8, 50.5, -163.9, -10.3, -20.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C11 I
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[466.5, -72.9, 53.1, -164.2, -9.6, -20.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[466.5, -82.4, 53.1, -164.2, -9.6, -20.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[470.0, -82.0, 50.5, -163.9, -10.3, -20.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 20
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_position(*[470.0, -115.2, 50.5, -163.9, -10.3, -20.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C15 I
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[466.5, -116.4, 53.1, -164.2, -9.6, -20.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[466.2, -16.0, 49.7, -157.5, -0.2, -18.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[457.6, -14.1, 50.4, -157.5, 4.5, -18.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C21 I
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[457.6, -14.1, 59.2, -157.5, 4.5, -18.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[466.2, -14.8, 59.5, -157.5, -0.2, -18.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[464.7, -138.2, 54.2, 151.7, -17.4, -18.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[464.7, -138.2, 51.0, 151.7, -17.4, -18.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[459.1, -138.2, 51.0, 151.7, -17.4, -18.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[456.0, -138.2, 51.0, 151.7, -12.3, -18.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C12 I
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[456.0, -135.7, 54.6, 151.7, -12.3, -18.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[452.6, -132.6, 54.5, 151.7, -1.3, -18.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[450.5, -130.5, 48.3, 151.7, -1.3, -18.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 20
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_position(*[450.5, -123.9, 48.3, 151.7, -1.3, -18.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # C11 I
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[452.6, -130.3, 56.1, 151.7, -1.3, -18.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[466.1, -130.1, 48.3, 155.1, 23.8, 60.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        self._tcp_speed = 20
                        code = self._arm.set_position(*[466.1, -123.0, 48.3, 155.0, 23.8, 60.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
            if self._vars.get('flag_d', 0):
                if True:
                    # drop
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[458.6, -132.2, 121.3, 155.0, 23.8, 60.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[458.0, -130.6, 124.9, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[451.4, -134.4, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[428.1, -135.2, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if False:
                    # drop
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[464.4, -135.2, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[478.6, -135.2, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # drop
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[454.6, -135.2, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[450.0, -75.0, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[427.2, -75.0, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # drop
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[465.3, -75.2, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[487.8, -75.2, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # drop
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[534.0, -75.2, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # drop
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[463.9, -75.2, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[463.9, -14.4, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[480.7, -14.4, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # drop
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[449.4, -14.4, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[428.0, -14.0, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # drop
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[451.7, -13.9, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[450.4, 46.4, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[428.5, 46.4, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    # drop
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[454.6, 46.4, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[474.0, 46.4, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
            if True:
                if True:
                    # -
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[638.5, 46.4, 131.7, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[614.1, -180.2, 97.0, 179.9, -1.8, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                if self._vars.get('flag_m', 0):
                    # C1234
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[581.2, -213.9, 56.6, -156.7, 33.2, 82.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[577.3, -212.9, 51.5, -156.7, 33.2, 82.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 15
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_position(*[578.1, -170.8, 50.0, -156.7, 33.2, 82.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[576.1, -173.8, 48.2, 156.6, 21.0, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[547.7, -173.8, 47.0, 156.6, 21.0, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[545.6, -173.8, 78.9, 156.6, 21.0, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[545.6, -283.1, 78.9, 156.6, 21.0, 82.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[543.9, -265.2, 95.5, -152.4, -24.5, 18.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[554.8, -180.0, 54.2, -152.4, -24.5, 18.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[559.5, -180.0, 49.5, -152.4, -24.5, 18.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 1, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        self._tcp_speed = 15
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                        code = self._arm.set_position(*[559.5, -223.9, 49.5, -152.4, -24.5, 18.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                    if True:
                        self._tcp_speed = 500
                        self._tcp_acc = 5000
                        code = self._arm.set_position(*[553.0, -266.4, 57.7, -160.0, -18.5, 18.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[576.0, -220.4, 56.4, -160.0, -31.2, 18.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_position(*[576.8, -216.6, 53.0, -160.0, -31.2, 18.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
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
                        code = self._arm.set_position(*[554.6, -216.6, 53.0, -160.0, -31.2, 18.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                        if not self._check_code(code, 'set_position'):
                            return
                        code = self._arm.set_cgpio_digital(7, 0, delay_sec=0, sync=True)
                        if not self._check_code(code, 'set_cgpio_digital'):
                            return
                        for i in range(int(0.5/ 0.1)):
                            time.sleep(0.1)
                            if not self.is_alive:
                                return
                if True:
                    self._tcp_speed = 500
                    self._tcp_acc = 5000
                    code = self._arm.set_position(*[554.6, -260.6, 67.5, -160.0, -31.2, 18.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[520.8, -277.9, 58.0, -159.4, -6.9, 18.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[262.0, -277.9, 58.0, -159.4, -6.9, 18.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[245.6, 101.8, 57.7, -159.7, -6.8, 38.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    code = self._arm.set_position(*[238.5, 145.5, 99.8, -176.9, 0.3, 88.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
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
