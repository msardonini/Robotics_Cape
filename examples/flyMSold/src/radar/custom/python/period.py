#!/usr/bin/python

from __future__ import print_function
import sys
from pprint import pprint
pprint(sys.path)
from pymavlink import mavutil
from pymavlink.dialects.v20 import radar as mavlink
import threading
import time
import monotonic
import datetime
import atexit

'''
test mavlink messages
'''

#from pymavlink.generator import mavtestgen

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--baudrate", type=int,
                  help="master port baud rate", default=115200)
parser.add_argument("--device", required=True, help="serial device")
parser.add_argument("--source-system", dest='SOURCE_SYSTEM', type=int,
                  default=255, help='MAVLink source system for this GCS')
args = parser.parse_args()

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    msg = m.recv_match(type='HEARTBEAT', blocking=True)
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

class ShowMessage(threading.Thread):
    def __init__(self, m):
        super(ShowMessage, self).__init__()
        self.m = m
        self.stop_event = threading.Event()
        self.setDaemon(True)
        self.last = [0 for i in range(256)]
        self.maxp = [0 for i in range(256)]
        self.minp = [0 for i in range(256)]
        self.period = [0 for i in range(256)]
        self.count = [0 for i in range(256)]

    def stop(self):
        for i in range(0, 256):
            if self.period[i] != 0:
                print(mavlink.mavlink_map[i], self.period[i], self.minp[i], self.maxp[i]) 
        self.stop_event.set()

    def run(self):
        global radar_param
        '''show incoming mavlink messages'''
        i = 0
        while not self.stop_event.is_set():
            msg = self.m.recv_match(blocking=True)
            if not msg:
                return
            if msg.get_type() == "BAD_DATA":
                if mavutil.all_printable(msg.data):
                    sys.stdout.write(msg.data)
                    sys.stdout.flush()
            else:
                msgid = msg.get_msgId()
                currenttime = monotonic.monotonic()
                c = self.count[msgid]
                p = self.period[msgid]
                l = self.last[msgid]
                d = currenttime - l
                if (self.last[msgid] != 0) :
                    self.maxp[msgid] = d if d > self.maxp[msgid] else self.maxp[msgid]
                    self.minp[msgid] = d if ((d < self.minp[msgid]) or (self.minp[msgid] == 0)) else self.minp[msgid]
                    self.period[msgid] = ((c * p) + d)/(c + 1)
                ++self.count[msgid]
                self.last[msgid] = currenttime
                print(i, msg, self.period[msgid])
                i += 1
                if msg.get_msgId() == mavlink.MAVLINK_MSG_ID_RADAR_PARAM:
                    radar_param.set(msg)

th_me = None                        
def exit_thread():
    print("called exit_thread()")
    th_me.stop()

def main():
    global th_me
    global radar_param
    # create a mavlink serial instance
    master = mavutil.mavlink_connection(args.device, baud=args.baudrate, source_system=args.SOURCE_SYSTEM, dialect="radar")
    
    # wait for the heartbeat msg to find the system ID
    wait_heartbeat(master)
    th_me = ShowMessage(master);
    atexit.register(exit_thread, )
    th_me.start()

    master.mav.param_set_send(1, 1, "RDAR_LOGD_P", 1, mavlink.MAV_PARAM_TYPE_INT32)

    while 1:
    	time.sleep(1)

    th_me.stop()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("KeyboardInterrrupt")
        exit_thread()
        exit()

