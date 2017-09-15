#!/usr/bin/python

from __future__ import print_function
import sys
from pprint import pprint
pprint(sys.path)
from pymavlink import mavutil
from pymavlink.dialects.v20 import radar as mavlink
import threading
import time
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

class RadarParam():
    initialized = 0
    sensing_mode = 0
    data_sel = 0
    decimate = 0
    rx_gain = 0
    fout_ch1 = 0
    fmcw_t = 0
    fmcw_f = 0 
    fmcw_fast = 0
    alpha = 0.0
    weight = 0
    thrsh_db = 0
    cal_period = 0
    lower = 0
    upper = 0
    search = 0
    window_func = 0
    dc_cut = 0
    output = 0
    nfft = 0
    period = 0
    enlpf = 0

    def send(self, mavlink):
        if (self.initialized):
            print(vars(self))
            mavlink.radar_param_send(self.sensing_mode, self.data_sel, self.decimate, self.rx_gain, self.fout_ch1, self.fmcw_t, self.fmcw_f, self.fmcw_fast, self.alpha, self.weight, self.thrsh_db, self.cal_period, self.lower, self.upper, self.search, self.window_func, self.dc_cut, self.output, self.nfft, self.period, self.enlpf)

    def set(self, msg):
        self.initialized = 1
        self.sensing_mode = msg.sensing_mode
        self.data_sel = msg.data_sel
        self.decimate = msg.decimate
        self.rx_gain = msg.rx_gain
        self.fout_ch1 = msg.fout_ch1
        self.fmcw_t = msg.fmcw_t
        self.fmcw_f = msg.fmcw_f
        self.fmcw_fast = msg.fmcw_fast
        self.alpha = msg.alpha
        self.weight = msg.weight
        self.thrsh_db = msg.thrsh_db
        self.cal_period = msg.cal_period
        self.lower = msg.lower
        self.upper = msg.upper
        self.search = msg.search
        self.window_func = msg.window_func
        self.dc_cut = msg.dc_cut
        self.output = msg.output
        self.nfft = msg.nfft
        self.period = msg.period
        self.enlpf = msg.enlpf

radar_param = RadarParam()

class ShowMessage(threading.Thread):
    def __init__(self, m):
        super(ShowMessage, self).__init__()
        self.m = m
        self.stop_event = threading.Event()
        self.setDaemon(True)

    def stop(self):
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
                print(i, msg)
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
    #master.mav.param_set_send(1, 1, "MAV_PROTO_VER", 2, mavlink.MAV_PARAM_TYPE_INT32)
    #master.mav.param_request_read_send(1, 1, "MAV_PROTO_VER", -1)
    
    #show_messages(master)
    
    #th_me = threading.Thread(target=show_messages, name="show_messages", args=(master,))
    th_me = ShowMessage(master);
    atexit.register(exit_thread, )
    th_me.start()

#    while 1:
#    	time.sleep(1)

    master.mav.radar_command_send(mavlink.RADAR_CMD_START, 0, 0)
    master.mav.radar_command_send(mavlink.RADAR_CMD_DUMP_PARAM, 0, 0)
    logging_radar = 0

    while 1:
#    for i in range(0, 1000):
#        master.mav.radar_command_send(mavlink.RADAR_CMD_STOP, 0, 0)
#        print("send radar_cmd stop")
#        time.sleep(5)
        master.mav.radar_command_send(mavlink.RADAR_CMD_DUMP_PARAM, 0, 0)
        print("send radar_cmd dump_param");
        time.sleep(5)
        master.mav.param_set_send(1, 1, "RDAR_LOGD_P", logging_radar, mavlink.MAV_PARAM_TYPE_INT32)
        if logging_radar == 0:
            logging_radar = 1
        else:
            logging_radar = 0
#        print("radar_param.enlpf", radar_param.enlpf)
#        if radar_param.enlpf == 0:
#            radar_param.enlpf = 1
#        else :
#            radar_param.enlpf = 0
#        print("radar_param.enlpf", radar_param.enlpf)
#        radar_param.send(master.mav)
#        print("send radar_param")
#        time.sleep(5)
#        master.mav.radar_command_send(mavlink.RADAR_CMD_START, 0, 0)
#        print("send radar_cmd start")
        time.sleep(5)

    th_me.stop()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("KeyboardInterrrupt")
        exit_thread()
        exit()

