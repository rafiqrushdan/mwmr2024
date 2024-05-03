#!/usr/bin/env python3
from termcolor import colored
from dataclasses import dataclass
from threading import Thread
import time

import sys
import select
import tty
import termios

import rospy
from std_msgs.msg import Int16, Float32
from typing import List


def kbhit():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


old_settings = termios.tcgetattr(sys.stdin)


@dataclass
class Display:
    motor_id = 0
    Kp = 1.0
    Ki = 0.1
    Kd = 0.5
    speed = [0, 0, 0, 0]

    selector = 0
    current_text = ''

    def selector_text(self, text: str, id: int):
        return colored(text, "black" if self.selector == id else "white", "on_white" if self.selector == id else "on_black")

    def show_current_text(self, text: str, id: int):
        if self.selector != id:
            return text
        if not self.current_text:
            return text
        return f'{self.current_text:>10}'

    def selector_handler(self, ):
        if self.current_text:
            if self.selector == 1:
                self.Kp = float(self.current_text)
            if self.selector == 2:
                self.Ki = float(self.current_text)
            if self.selector == 3:
                self.Kd = float(self.current_text)
            if self.selector == 4:
                self.speed[self.motor_id] = float(self.current_text)
        self.current_text = ''

    def display(self):
        global thread_stop
        while not thread_stop:
            print('\r', end='')
            print(self.selector_text(f'motor{self.motor_id+1}', 0), end=' ')
            print('Kp:', self.selector_text(
                self.show_current_text(f'{self.Kp:>10}', 1), 1), end=' ')
            print('Ki:', self.selector_text(
                self.show_current_text(f'{self.Ki:>10}', 2), 2), end=' ')
            print('Kd:', self.selector_text(
                self.show_current_text(f'{self.Kd:>10}', 3), 3), end=' ')
            print('Speed:', self.selector_text(
                self.show_current_text(f'{self.speed[self.motor_id]:>10}', 4), 4), end=' ')

            if kbhit():
                keypress = sys.stdin.read(1)
                keypress = int.from_bytes(keypress.encode(), byteorder='big')
                if chr(keypress) == 'd':
                    self.selector_handler()
                    self.selector += 1
                elif chr(keypress) == 'a':
                    self.selector_handler()
                    self.selector -= 1

                elif 48 <= keypress <= 57 or chr(keypress) == '.':
                    self.current_text += chr(keypress)
                elif keypress == 8:  # backspace
                    self.current_text = self.current_text[:-1]

                elif chr(keypress) == 'w' and self.selector == 0:
                    self.motor_id -= 1
                elif chr(keypress) == 's' and self.selector == 0:
                    self.motor_id += 1

            self.selector = self.selector % 5
            self.motor_id = self.motor_id % 4


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    value: float = 0
    target: float = 0
    _i: float = 0
    error_prev: float = 0
    time_prev: float = time.time()

    @property
    def time_diff(self):
        return time.time() - self.time_prev

    @property
    def error(self):
        return self.target - self.value

    @property
    def p(self):
        return self.kp * self.error

    @property
    def i(self):
        self._i = self._i + self.ki * self.error
        return self._i

    @property
    def d(self):
        return self.kd*((self.error - self.error_prev)/self.time_diff)

    @property
    def total(self):
        pid = self.p + self.i + self.d
        self.error_prev = self.error
        self.time_prev = time.time()
        return pid

motor_pubs: List[rospy.Publisher] = []
enc_subs: List[rospy.Subscriber] = []
encoder_values = [0 for _ in range(4)]
pids: List[PID] = []

def shutdown_hook():
    for pub in motor_pubs:
        pub.publish(Int16(0))

def encoder_callback(msg: Float32, id: int):
    encoder_values[id] = msg.data
        


thread_stop = False
display = Display()

if __name__ == '__main__':
    rospy.init_node('test_pid')
    rospy.loginfo('Node started')

    tty.setcbreak(sys.stdin.fileno())

    thread1 = Thread(target=display.display)

    thread1.start()

    
    for i in range(4):
        pub = rospy.Publisher(f'motor{i+1}', Int16, queue_size=10)
        motor_pubs.append(pub)
        sub = rospy.Subscriber(f'enc{i+1}', Float32,
                                callback=encoder_callback,
                                callback_args=i+1)
        enc_subs.append(sub)
        pid = PID(display.Kp, display.Ki, display.Kd)
        pids.append(pid)

    rospy.on_shutdown(shutdown_hook)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
            for i in range(4):
                pids[i].kp = display.Kp
                pids[i].ki = display.Ki
                pids[i].kd = display.Kd
                pids[i].target = display.speed[i]
                pids[i].value = encoder_values[i]

                msg = Int16(int(pids[i].total))
                motor_pubs[i].publish(msg)
                rate.sleep()
    thread_stop = True
    thread1.join()
    print()
