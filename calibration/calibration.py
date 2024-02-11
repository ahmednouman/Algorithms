#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import libevdev
import time
from random import randrange
import random
import evdev
import numpy as np
from evdev import UInput, AbsInfo, ecodes, categorize 
import signal
import os, time


path_to_calibration = "/home/espresso/multiEspresso"
path_to_ctm_query = "/home/espresso/query_ctm"
path_to_write_ctm_txt = "/home/espresso/simple/calibration/CTM.txt"
os.system(path_to_calibration)
os.system(path_to_ctm_query+" "+path_to_write_ctm_txt)

def getCTM():
    with open(path_to_write_ctm_txt, "r") as f:
        values=[]
        k=0
        for i,l in enumerate(f):
            s = l.split("\n")[0]
            values.append(float(s))
            
    CTM = np.reshape(values,(3,3))
    print(CTM)
    return CTM

def handler(signum, frame):
    global filter, save, angle
    res = input("want to plot? y/n")
    if res == "y":
        # if filter_type == "simple":
        print(len(speed.speedavg), len(speed.speedmed), len(angle.savedAngles))
        save.writeToFile(speed.speedavg, speed.speedmed, angle.savedAngles)
        # else:
        #     save.writeToFile( angle.savedAngles)
    exit(1)

signal.signal(signal.SIGINT, handler)

def main(args):
    global filter, save, angle
    dev = libevdev.Device()
    dev.name = "Wacom TouchScreen Pen stylus"

    dev.enable(libevdev.INPUT_PROP_DIRECT)
    # Say that we are using the pen (not the erasor), and should be set to 1 when we are at proximity to the device.
    # See http://www.infradead.org/~mchehab/kernel_docs_pdf/linux-input.pdf page 9 (=13) and guidelines page 12 (=16), or the https://github.com/linuxwacom/input-wacom/blob/master/4.5/wacom_w8001.c (rdy=proximity)
    dev.enable(libevdev.EV_KEY.BTN_TOOL_PEN)
    dev.enable(libevdev.EV_KEY.BTN_TOOL_RUBBER)
    # Click
    dev.enable(libevdev.EV_KEY.BTN_TOUCH)
    # Press button 1 on pen
    dev.enable(libevdev.EV_KEY.BTN_STYLUS)
    # Pressstate_init_flag button 2 on pen, see great doc
    dev.enable(libevdev.EV_KEY.BTN_STYLUS2)
    # Send absolute X coordinate
    dev.enable(libevdev.EV_ABS.ABS_X,
               libevdev.InputAbsInfo(minimum=0, maximum=16383, resolution=100))
    # Send absolute Y coordinate
    dev.enable(libevdev.EV_ABS.ABS_Y,
               libevdev.InputAbsInfo(minimum=0, maximum=9599, resolution=100))
    # Send absolute pressure
    dev.enable(libevdev.EV_ABS.ABS_PRESSURE,
               libevdev.InputAbsInfo(minimum=0, maximum=4095))
    # Use to confirm that we finished to send the informations
    # (to be sent after every burst of information, otherwise
    # the kernel does not proceed the information)
    dev.enable(libevdev.EV_SYN.SYN_REPORT)
    # Report buffer overflow
    dev.enable(libevdev.EV_SYN.SYN_DROPPED)


    devices = evdev.list_devices()
    p = 0
    i = 0            
    for dev_sp in devices:
        print('%-12i%s' % (p, evdev.InputDevice(dev_sp).name))
        if evdev.InputDevice(dev_sp).name == 'Wacom TouchScreen Pen':
            i = p
        p += 1

    device = evdev.InputDevice(devices[i])
    device.grab()

    uinput = dev.create_uinput_device()
    print("New device at {} ({})".format(uinput.devnode, uinput.syspath))

    CTM = getCTM()

    touched = False
    released = False

    no_filter = False

    x_prev = 0
    y_prev = 0

    x_fil_prev = 0
    y_fil_prev = 0

    ts_x_prev = 0
    ts_y_prev = 0

    speed_x_prev = 0
    speed_y_prev = 0

    state_init_flag = False

    updated_pt_x = 0
    updated_pt_y = 0

    for event in device.read_loop():
        code, val = event.type, event.value
        q = str(categorize(event))
        # import pdb; pdb.set_trace()
        if "ABS_X" in q:
                calibrated_x = np.dot(CTM, [[val], [y_prev], [1]])[0][0] 
                updated_pt_x = round(calibrated_x)
                uinput.send_events([
                    libevdev.InputEvent(libevdev.EV_ABS.ABS_X,
                                        value=updated_pt_x)]) 
                         
        elif "ABS_Y" in q:
            if(val != 0):
                calibrated_y = np.dot(CTM, [[x_prev], [val], [1]])[1][0] 
                updated_pt_y = round(calibrated_y)
                uinput.send_events([
                    libevdev.InputEvent(libevdev.EV_ABS.ABS_Y,
                                        value=updated_pt_y)])
                global_y  = val
                    
            
        elif "BTN_TOUCH" in q:
            if val == 1:

                touched = True
                released = False

            else:
       
                touched = False
                released = True
                state_init_flag = False
                x_prev = 0
                y_prev = 0

            uinput.send_events([
                libevdev.InputEvent(libevdev.EV_KEY.BTN_TOUCH,
                                    value=val)
            ])            
        elif "BTN_TOOL_PEN" in q:
            #print("TOOL_PEN", val)
            uinput.send_events([
                libevdev.InputEvent(libevdev.EV_KEY.BTN_TOOL_PEN,
                                    value=val)
            ])
            # if val == 1:
            #     touched = True
            #     released = False
            # else:
            #     touched = False
            #     released = True
            #     touch_iter = 0
            #     data_x = []
            #     data_y = [] 
                  
        elif "BTN_STYLUS" in q:
            print("BTN_STYLUS")
            uinput.send_events([
                libevdev.InputEvent(libevdev.EV_KEY.BTN_STYLUS,
                                    value=val)])

        elif "BTN_STYLUS2" in q:
            print("BTN_STYLUS2")
            uinput.send_events([
                libevdev.InputEvent(libevdev.EV_KEY.BTN_STYLUS,
                                    value=val)
            ]) 
        elif "ABS_PRESSURE" in q:
            #print("ABS_PRESSURE")
            uinput.send_events([
                libevdev.InputEvent(libevdev.EV_ABS.ABS_PRESSURE,
                                    value=val)
            ])  
        elif "SYN_REPORT" in q:
                  uinput.send_events([libevdev.InputEvent(libevdev.EV_SYN.SYN_REPORT,
                                    value=0)])

        # else:
        #     print(categorize(event))



if __name__ == "__main__":
    if len(sys.argv) > 2:
        print("Usage: {}")
        sys.exit(1)
    main(sys.argv)
