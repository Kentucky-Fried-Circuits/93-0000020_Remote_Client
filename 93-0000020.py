#!/usr/bin/env python3
# reference implementation for the Solar Stik Modbustik protocol.
# see RELEASE.txt for release notes and known issues
__author__ = "Brian Alano"
__license__ = "(C) 2020-2022 Solar Stik, Inc. All rights reserved."
__status__ = "Development"
__url__ = "https://www.solarstik.com"
__version__ = "V1.4.0"

#####################################################################################
# Configuration parameters

debug = False
address = 1
log_interval = 80  # ms between log reads
can_transmit_interval = 0.1  # seconds
log_handle = None
write_count = 0

#####################################################################################
# Coding conventions
# Error codes from linux error code https://kdave.github.io/errno.h/, just because
#####################################################################################

import sys
import threading
import time
import datetime
import PySimpleGUI as sg
import minimalmodbus
import math
import serial.tools.list_ports  # from pyserial
import csv
import logging
# import can.interfaces.pcan.basic
# import can.interfaces.pcan.pcan

import j1939  # package can-j1939

# name descriptor for this CAN Controller Application. Arbitrary (therefore non-conformal) because we aren't subscribers.
name = j1939.Name(
    arbitrary_address_capable=1,
    industry_group=j1939.Name.IndustryGroup.Industrial,
    vehicle_system_instance=1,
    vehicle_system=1,
    function=1,
    function_instance=1,
    ecu_instance=1,
    manufacturer_code=666,
    identity_number=1234567
)

logging.getLogger('j1939').setLevel(logging.DEBUG)
logging.getLogger('can').setLevel(logging.DEBUG)

# TODO
#  create the ControllerApplications
# ca = j1939.ControllerApplication(name, 128)

# list of tuples containing all product-specific information:
#   product name, baud rate, Modbus address, register dictionary, modbus write function code (0x06 and 0x10 supported)
products = [
    (
        '24VDC HyPR 6000 (20-0104024)',
        57600,
        0x1,
        {  # must match register list in HyPR 6000 firmware's RMAGS.h
            "BIT": 0,
            "TOTAL_ERRORS": 1,
            "LOG_HEAD": 2,
            "LOG_TAIL": 3,
            "LOG_STATUS": 4,
            "LOG_LEVEL": 5,
            "LOG_CODE": 6,
            "LOG_DEVICE": 7,
            "LOG_MSG_LEN": 8,  # in bytes, while LOG_VALUEn is in 16-bit words
            "LOG_VALUE0": 9,
            "LOG_VALUE1": 10,
            "LOG_VALUE2": 11,
            "LOG_VALUE3": 12,
            "LOG1_LEVEL": 13,
            "LOG1_CODE": 14,
            "LOG1_DEVICE": 15,
            "LOG1_MSG_LEN": 16,  # in bytes, while LOG_VALUEn is in 16-bit words
            "LOG1_VALUE0": 17,
            "LOG1_VALUE1": 18,
            "LOG1_VALUE2": 19,
            "LOG1_VALUE3": 20,
            "LOG2_LEVEL": 21,
            "LOG2_CODE": 22,
            "LOG2_DEVICE": 23,
            "LOG2_MSG_LEN": 24,  # in bytes, while LOG_VALUEn is in 16-bit words
            "LOG2_VALUE0": 25,
            "LOG2_VALUE1": 26,
            "LOG2_VALUE2": 27,
            "LOG2_VALUE3": 28,
            "LOG3_LEVEL": 29,
            "LOG3_CODE": 30,
            "LOG3_DEVICE": 31,
            "LOG3_MSG_LEN": 32,  # in bytes, while LOG_VALUEn is in 16-bit words
            "LOG3_VALUE0": 33,
            "LOG3_VALUE1": 34,
            "LOG3_VALUE2": 35,
            "LOG3_VALUE3": 36,
            "START_VOLTAGE": 37,  # in centivolts
            "START_DELAY": 38,  # in s
            "STOP_VOLTAGE": 39,  # in centivolts
            "STOP_DELAY": 40,  # in s
            "CRANK_TIMEOUT": 41,  # in s
            "REGULATED_VOLTAGE": 42,  # in centivolts
            "MAX_CHARGING_CURRENT": 43,  # in A
            "LVCO": 44,  # in centivolts
            "BUS_VOLTAGE": 45,  # in centivolts
            "CURRENTS": 46,  # MSB - charger current, LSB - regulator current, in A
            "GEN_CONTROL": 47,
            "AGS_STATUS": 48,
            "AC_INPUT": 49,
            # 0 for not sensed, 1 for sensed.Used to detect if generator is running.Future use may include voltage and / or frequency
            "AMMPS_RESPONSE": 50,  # The first 6 bits of PGN 0xFF17.The MSB is 0.
            "GEN_WARMUP": 51,  # in s
            "WARMUP_STATE": 52,  # 0 - NoAC, 1 - WarmingUp, 2 - Running
            "GENERATOR_POWER_SETTINGS": 53,  # watts. 1000 to 5000 watts by 100s
            "AVAILABLE_CURRENT": 54,  # in A. available current given available power
            "CHARGER_MODE": 55,  # 0 - CC, 1 - CV, 2 - Off, 3 - Delay
            "PID_KP": 56,  # Charger proportional gain
            "PID_KI": 57,  # Charger integral gain
            "PID_KD": 58,  # Charger derivative gain
            "PID_INPUT": 59,  # pid input current * 256 (same as unregulated current)
            "PID_OUTPUT": 60,  # pid output current * 256 (current requested by PID)
            "PID_SET_POINT": 61,
            # pid set point current * 256 (the minimum of available_current, MAX_CHARGE_CURRENT, and charging_current)
            "CHARGING_CURRENT_OFFSET": 62,  # command to zero calibrate the current sensors
            "REGULATED_CURRENT_OFFSET": 63,  # command to zero calibrate the current sensors
            "VERSION": 64,  # firmware version encoded as 16-bit number
            # a hack to tell us the total number of registers. Function 3 and 16 share the same register array
            "TOTAL_REGS_SIZE": 65
        },
        0x10,
    ),
    (
        '24VDC PRO-Verter 5000-220 AFF1 (20-0104033)',
        9600,
        0x1,
        {  # must match register list in the EVO series - MODBUS communications protocol document
            'Absorb time': 0x2E,
            'Absorb exit amps': 0x2F,
            'Bulk current': 0x30,
            'Absorb voltage': 0x31,
            'Equalization voltage': 0x32,
            'Floating voltage': 0x33,
            'Temperature compensation': 0x34,
            'Battery low voltage': 0x35,
            'Battery over voltage': 0x36,
            'Low voltage alarm': 0x37,
            'Reset voltage': 0x38,
            'Low voltage detect time': 0x39,
            'Low voltage cut off time': 0x3A,
            'Charge mode (equalization)': 0x3B,
            'Mode (Online mode)': 0x3C,
            'Online option': 0x3D,
            'Reset to Bulk stage': 0x3E,
            'Charging profile': 0x3F,
            'Default frequency': 0x40,
            'Grid input max current': 0x41,
            'Low frequency cut off': 0x42,
            'Low frequency reset': 0x43,
            'High frequency cut off': 0x44,
            'High frequency reset': 0x45,
            'Low voltage reset': 0x46,
            'Low voltage cut off 1': 0x47,
            'Low voltage cut off 2': 0x48,
            'Low voltage cut off 3': 0x49,
            'Low voltage detect time 1': 0x4A,
            'Low voltage detect time 2': 0x4B,
            'Low voltage detect time 3': 0x4C,
            'High voltage reset': 0x4D,
            'High voltage cut off 1': 0x4E,
            'High voltage cut off 2': 0x4F,
            'High voltage cut off 3': 0x50,
            'High voltage detect time 1': 0x51,
            'High voltage detect time 2': 0x52,
            'High voltage detect time 3': 0x53,
            'Gen input maximum current': 0x54,
            'GS detect time': 0x55,
            'GEN ON time': 0x56,
            'GEN OFF delay': 0x57,
            'Battery type': 0x58,
            'Input OC Protection': 0x59,
            'Input Recovery': 0x5A,
            'Sync. Grid': 0x5B,
            'Sync. GEN': 0x5C,
            'Safe Charging': 0x5D,
            'External Charger': 0x5F,
            'Power saving': 0x60,
            'Enter point': 0x61,
            'Wake up point': 0x62,
            'Remote switch': 0x63,
            'Relay function': 0x64,
            'Comm. ID': 0x65,
            'Buzzer': 0x66,
            'Discharge beep': 0x67,
            'Default reset': 0x68,
            'Data log time': 0x69,
            'Temperature unit': 0x6B,
            'Password disable': 0x6C,
            'Remote switch delay time': 0x6D,
            'Status of GEN input': 0x100,
            'Frequency of GEN input': 0x101,
            'Voltage of GEN input': 0x102,
            'Status of Grid input': 0x103,
            'Frequency of Grid input': 0x104,
            'Voltage of Grid input': 0x105,
            'Input current': 0x106,
            'Input VA': 0x108,
            'Input watt': 0x10A,
            'Output frequency': 0x10C,
            'Output voltage': 0x10D,
            'Invert/Charge current': 0x10E,
            'Invert/Charge VA': 0x110,
            'Invert/Charge watt': 0x112,
            'Battery voltage': 0x114,
            'Battery current': 0x115,
            'External current': 0x116,
            'Battery temperature': 0x117,
            'Transformer temperature': 0x118,
            'Bus bar temperature': 0x119,
            'Heat sink temperature': 0x11A,
            'Fan speed': 0x11B,
            'Operating mode': 0x11C,
            'Error code': 0x11D,
            'Charge stage': 0x11E,
            'Version': 0x11F,
            'Compensating voltage': 0x120,
            'Control': 0x200,
        },
        0x06,
    ),

]

# some globals that should be replaced with better coding practices
instrument = None
windows = None
product = 0  # initial selection
comm_status = ''

# TODO CAN bus stuff
# register values in converted (semantic) format, keyed by register number
reg_vals = {
    59: None,
    60: None,
    61: None
}
BIT_EXECUTE = 0x0001
LOG_ENTRY_SIZE = 8
log_levels = ["EMERG", "ALERT", "CRIT", "ERR", "WARNING", "NOTICE", "INFO", "DEBUG",
              "UNDEF", "UNDEF", "UNDEF", "UNDEF", "UNDEF", "UNDEF", "UNDEF", "NULL"]
devices = ['CAN_A', 'CAN_B', 'RS485']

# PGNs
pgn_names = {0: 'null',
             1: 'ADDRESS_CLAIMED',
             2: 'J1939_STATUS',
             0xFECA: 'Active DTIC',
             0xFF17: 'AdvDCS Response',
             0xFF15: 'Controller 1st PGN',
             0xFF19: 'Controller 2nd PGN',
             0xFF22: 'Controller 3rd PGN',
             0xFF23: 'Modbus Registers'
             }
# reverse PGNs dict
pgn_vals = {}
for key, value in pgn_names.items():
    pgn_vals[value] = key

# Modbus decoding
j1939_statuses = {
    0: 'ADDRESSCLAIM_INIT',
    1: 'ADDRESSCLAIM_INPROGRESS',
    2: 'NORMALDATATRAFFIC',
    3: 'ADDRESSCLAIM_FAILED'
}

charger_modes = {
    0: 'Charging',
    1: 'Standby',
    2: 'Off',
    3: 'Delay'
}

genset_statuses = {
    0: 'NOT READY TO CRANK',
    1: 'READY TO CRANK',
    2: 'DELAY TO CRANK - WTR KIT',
    3: 'DELAY TO CRANK - GLW PLG',
    4: 'CRANK',
    5: 'RUNNING',
    6: 'EMERGENCY STOP',
    7: 'IDLE MODE',
    8: 'POWERING DOWN',
    9: 'FACTORY TEST',
    10: 'DELAY TO CRANK - WINTER KIT TEST',
    11: 'DELAY TO CRANK - WINTER KIT DETECT',
    12: 'DELAY TO CRANK - ECM DATASAVE',
    13: 'RUNNING - SYNCHRONIZING',
    14: 'RUNNING - SYNCHRONIZED',
    15: 'RUNNING - LOAD SHARE'
}

control_switch_positions = {
    0: 'Off',
    1: 'Prime and run Aux fuel',
    2: 'Prime and run',
    3: 'Start'
}

warmup_states = {
    0: 'NoAC',
    1: 'WarmingUp',
    2: 'Running'
}

ac_inputs = {
    0: 'Not sensed',
    1: 'Sensed'
}

gen_controls = {
    0: 'Off',
    1: 'Auto',
    5: 'On'
}

# PLot stuff
import random
from itertools import count
# import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits import mplot3d

plt.style.use('fivethirtyeight')
x_values = []
pid_inputs = []
pid_outputs = []
pid_set_points = []
counter = 0
index = count()
fig = plt.figure()
ax = plt.axes()

normal_delay = 0.0  # .03 s seems slow enough. .02 s is too fast
minimum_delay = 0.01
showlogs = False  # flag to communicate with readLogs
values = {}
wait = 0.010  # seconds between log reads, auto adjusted to keep from reading empty logs and from letting log overflow

#####################################################################################

"""can_connect - initialize CAN bus communications
"""


def can_connect():
    global can_transmit_interval

    # create the ElectronicControlUnit (one ECU can hold multiple ControllerApplications)
    ecu = j1939.ElectronicControlUnit()
    bustype = 'pcan'
    channel = 'PCAN_USBBUS1'
    try:
        ecu.connect(bustype=bustype, channel=channel, bitrate=250000)
    except can.interfaces.pcan.pcan.PcanCanInitializationError as e:
        proceed = sg.popup_ok(f'Unable to connect to {bustype} on {channel}')
        return None
    # add CA to the ECU
    ecu.add_ca(controller_application=ca)
    ca.subscribe(ca_receive)
    # callback every so often to transmit
    ca.add_timer(can_transmit_interval, ca_timer_callback)
    # by starting the CA it starts the address claiming procedure on the bus
    ca.start()
    print("ca started")
    return True


# line - command to execute, as a list of parameters
# preserve - content of -OUTPUT- to not overwrite during an update to -OUTPUT-
def do_script_line(line, preserve):
    global comm_status
    """Process a line from the script"""
    ret = preserve
    try:
        addr = products[product][3][line[0]]
    except Exception as e:
        comm_status = f'Unexpected register name: {e}. *** FAIL ***'
        ret += comm_status
        window['-OUTPUT-'].update(ret)
        window['-COMMS-'].update(comm_status)
        window.refresh()
        return ret

    comm_status = ''
    val = line[1]
    write = False if len(line) < 3 else line[2]
    delay = 0 if len(line) < 4 else line[3]
    iterations = 1 if len(line) < 5 else line[4]
    desc = '' if len(line) < 6 or len(line[5].strip()) == 0 else line[5].strip()
    if len(desc) >= 7 and desc[0:7].lower() == "assert:":
        assertion = desc[7:].strip()
    else:
        assertion = ''
    desc = ': ' if len(desc) == 0 else f' [{desc}]: '
    # print(f'{line[5]}, {desc}, {assertion}')
    if write:
        ret += f'Write {val} to {line[0]}'
    else:
        if val > 1:  # read multiple registers
            ret += f'Read {val} registers starting at {line[0]}'
        else:
            ret += f'Read from {line[0]}'

    ret += desc

    for i in range(iterations):
        if iterations > 1:
            ret += f'{i + 1}:'
        if write:
            result = write_register(addr, val, desc)
            ret += result
        else:
            if val > 1:  # read_registers()
                try:
                    result = (read_registers(addr, val))
                except ValueError as e:
                    sg.popup("ValueError. Maybe you asked for too many registers?")
                    result = "(null)"
            else:  # read_register
                try:
                    result = (read_register(addr, 0))
                except minimalmodbus.IllegalRequestError as e:
                    sg.popup_timed("IllegalRequestError. Probably the register does not exist on the host.")
                    result = "(Null)"
            ret += str(result)
        if len(assertion) > 0:
            try:
                asserted = eval(assertion, {}, {"x": result})
            except (NameError, TypeError, SyntaxError) as e:
                ret += f': *** ERROR *** {e}'
            else:
                if asserted:
                    ret += ': PASS'
                else:
                    ret += ': *** FAIL ***'
        if i + 1 < iterations:
            ret += '; '
        window['-OUTPUT-'].update(ret)
        window['-COMMS-'].update(comm_status)
        window.refresh()
        time.sleep(delay)
        ret += '\n'

    window['-OUTPUT-'].update(ret)
    window['-COMMS-'].update(comm_status)
    window.refresh()

    return ret


"""Callback for sending messages

    FIXME: This callback is registered at the ECU timer event mechanism to be 
    executed every 500ms.

    :param cookie:
        A cookie registered at 'add_timer'. May be None.
    """


def ca_timer_callback(cookie):
    # wait until we have our device_address
    if ca.state != j1939.ControllerApplication.State.NORMAL:
        # returning true keeps the timer event active
        return True

    for register in [0x3b, 0x3c, 0x3d]:  # PID_INPUT, PID_OUTPUT and PID_SET_POINT request
        data = [0x00, register]
        # sending normal broadcast message
        ca.send_pgn(0, 0xFF, 0x23, 6, data)
        time.sleep(0.05)

    # returning true keeps the timer event active
    return True


"""ca_receive - callback to execute on receipt of a CAN message

:param int priority:
    Priority of the message
:param int pgn:
    Parameter Group Number of the message
:param int sa:
    Source Address of the message
:param int timestamp:
    Timestamp of the message
:param bytearray data:
    Data of the PDU
"""


def ca_receive(priority, pgn, sa, timestamp, data):
    # parse CAN messages

    if pgn == pgn_vals['Modbus Registers'] and len(data) >= 4:  # message contains a Modbus register value
        reg = data[0] * 256 + data[1]
        print('reg', reg)
        val = data[2] * 256 + data[3]  # convert to 16-bit unsigned int
        if reg in [regs["PID_INPUT"], regs["PID_OUTPUT"], regs["PID_SET_POINT"]]:
            val = val / 256  # convert to float
        try:
            reg_vals[reg] = val
            name = reg_names[reg]
        except KeyError as e:
            print(f'ca_receive(): KeyError reg_vals[{reg}]')
            raise e
        print(f'register {name}({reg}):{reg_vals[reg]}')
    else:
        print("PGN {} length {}".format(pgn, len(data)))


"""can_plot - visualize CAN data
Really shouldn't couple the CAN data with the graph, but it's expedient for what I'm trying to do
"""


def animate(i):
    plot_window = 60
    x = time.monotonic()
    counter = next(index)
    print(f'animate():{counter}')
    x_values.append(x)

    # append values to keep graph dynamic
    print('reg_vals[regs["PID_INPUT"]', reg_vals[regs["PID_INPUT"]])
    pid_inputs.append(reg_vals[regs["PID_INPUT"]])
    pid_outputs.append(reg_vals[regs["PID_OUTPUT"]])
    pid_set_points.append(reg_vals[regs["PID_SET_POINT"]])

    if counter > plot_window:
        x_values.pop(0)
        pid_inputs.pop(0)
        pid_outputs.pop(0)
        pid_set_points.pop(0)
        # counter = 0
        plt.cla()  # clears the values of the graph

    plt.plot(x_values, pid_inputs, 'blue', linestyle='solid')
    plt.plot(x_values, pid_outputs, 'red', linestyle='--')
    plt.plot(x_values, pid_set_points, 'green', linestyle=':')

    ax.legend(["PID Input", "PID Output", "PID Set Point"])
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Current (A)")
    plt.title('Telemetry')

    time.sleep(.1)  # keep refresh rate of 0.25 seconds


# read the log and accumulate in logs. Tried to make this a daemon, but failed
def readlog(values, connected, f):
    # global showlogs
    # global values
    # global wait
    logs = []
    #   while len(values) == 0: # wait for GUI
    #       time.sleep(1)
    #    while True:
    if values['-LOGGING-'] == True and connected:
        try:
            ret = read_registers(products[product][3]["LOG_LEVEL"], 8)
            # handling logging as a special case, because we don't care to see a bunch of empty reads
        except NameError as e:
            #               sg.popup("No connection")
            comm_status = 'Lost connection'
            print(comm_status)
            connected = False
        except serial.serialutil.SerialException as e:
            comm_status = f'Serial Error: {e}'
            print(comm_status)
            connected = False
        except Exception as e:
            comm_status = f'Unexpected error: {e}. Continuing on...'
            print(comm_status)
        else:
            # if we got a bad read or log is empty, do nothing, else print it and clear it
            comm_status = ''
            if ret is not None and ret[0] != 0x0F:
                logs.append(ret)
                #                 # there's data, so decrease the wait time
                #                 wait -= 0.001
                #             else:
                #                 # there's no data, so increase the wait time.
                #                 # If we increase the same amount we decrease,
                #                 # we'll end up reading an empty log 50% of the time
                #                 wait += 0.005
                #             wait = max(0, min(wait, 1))  # clamp to 0 to 1 second
                # check if the GUI wants us to show the log. This is bad form--should be in a separate thread
                #        if showlogs:
                for log in logs:
                    showlog(log, values['-TIMESTAMP-'], f)
        logs = []


#            showlogs = False
#          time.sleep(wait)
#     else:
#           time.sleep(0.1) # wait for connection

# showlog - send log entry to GUI and, optionally, to a file
#
# ret - list containing a single log entry to process
# show_time - boolean. Whether to show the time/date stamp.
def showlog(ret, show_time, log_handle):
    global write_count
    line = ""
    cline = ""
    red = ''  # didn't work-maybe need double escape red='\033[1;31;48m'
    black = ''  # '\033[1;30;48m'

    if show_time:
        line += f'{datetime.datetime.now().isoformat()}, '
        cline += f'{time.asctime()} '
    # decode
    if ret[0] > len(log_levels) - 1:
        line += f'{ret[0]}'
        cline += f'{red}{ret[0]}{black}'
    else:
        line += f'{log_levels[ret[0]]}'
        cline += f'{log_levels[ret[0]]}'
    code = pgn_names.get(ret[1])
    if code:
        line += f', {code}'
        cline += f', {code}'
    else:  # unknown code
        line += f", {hex(ret[1])}"
        cline += f", {red}{hex(ret[1])}{black}"
    # device and optional node address
    device = ret[2] & 0x00FF
    node = ret[2] >> 8
    if device > len(devices) - 1:
        line += f', illegel device: {device}'
        cline += f', {red}illegel device: {device}{black}'
    else:
        line += f', {devices[device]}'
        cline += f', {devices[device]}'
        if node > 0:
            line += f':{node:02x}'
            cline += f':{node:02x}'
    bytes = ret[3]
    if bytes <= 8:
        line += f', bytes:{bytes}'
        cline += f', bytes:{bytes}'
    else:
        line += f', bytes(illegal value, max 8):{bytes}'
        cline += f', {red}bytes(illegal value, max 8):{bytes}{black}'
        bytes = 8  # show the legal number
    if code == 'J1939_STATUS':
        j1939_status = j1939_statuses.get(ret[4])
        if j1939_status:
            line += f', {j1939_status}'
            cline += f', {j1939_status}'
        else:
            line += f", {hex(j1939_status)}"
            cline += f", {red}{hex(j1939_status)}{black}"
    elif code == 'Controller 1st PGN':
        generator_power_settings = ret[4]
        control_voltage = ret[5] >> 8
        max_charging_current = ret[5] & 0x00FF
        start_delay = ret[6] >> 8
        stop_delay = ret[6] & 0x00FF
        lvco = ret[7]
        line += f', GENERATOR_POWER_SETTINGS:{generator_power_settings}'
        line += f', CONTROL_VOLTAGE:{control_voltage}'
        line += f', MAX_CHARGING_CURRENT:{max_charging_current}'
        line += f', START_DELAY:{start_delay}'
        line += f', STOP_DELAY:{stop_delay}'
        line += f', LVCO:{lvco}'
        cline += f', GENERATOR_POWER_SETTINGS:{generator_power_settings}'
        cline += f', CONTROL_VOLTAGE:{control_voltage}'
        cline += f', MAX_CHARGING_CURRENT:{max_charging_current}'
        cline += f', START_DELAY:{start_delay}'
        cline += f', STOP_DELAY:{stop_delay}'
        cline += f', LVCO:{lvco}'
    elif code == 'Controller 2nd PGN':
        start_voltage = ret[4]
        stop_voltage = ret[5]
        unregulated_current = ret[6] >> 8
        regulated_current = ret[6] & 0x00FF
        regulated_voltage = ret[7]
        more = f', START_VOLTAGE:{start_voltage}'
        more += f', STOP_VOLTAGE:{stop_voltage}'
        more += f', UNREGULATED_CURRENT:{unregulated_current}'
        more += f', REGULATED_CURRENT:{regulated_current}'
        more += f', REGULATED_VOLTAGE:{regulated_voltage}'
        line += more
        cline += more
    elif code == 'Controller 3rd PGN':
        #    line += ', 3RD 3RD 3RD'
        #    cline += ', 3RD 3RD 3RD'
        battery_voltage = ret[4]
        charger_mode = charger_modes.get(ret[5] >> 8)
        bit_low_voltage_warning = 'Low voltage warning' if ret[5] & 0x0001 == 1 else ''
        bit_lvco = 'LVCO' if ret[5] & 0x0002 == 1 else ''
        bit = ';'.join([bit_low_voltage_warning, bit_lvco]).strip(';')
        charging_current = ret[6] >> 8
        gen_control = gen_controls.get(ret[6] & 0x00FF)
        more = f', BATTERY_VOLTAGE:{battery_voltage}'
        more += f', CHARGER_MODE:{charger_mode}'
        more += f', BIT:{bit}'
        more += f', CHARGING_CURRENT:{charging_current}'
        more += f', GEN_CONTROL:{gen_control}'
        line += more
        cline += more
    elif code == 'Modbus Registers':
        register = ';'.join([k for k, v in regs.items() if v == ret[4]])
        value = ret[5]
        if register == 'WARMUP_STATE':
            value = warmup_states.get(value)
        elif register == 'AC_INPUT':
            value = ac_inputs.get(value)
        more = f', {register}:{value}'
        line += more
        cline += more
    elif code == 'AdvDCS Response':
        genset_status = genset_statuses.get((ret[4] >> 8) & 0x0F)
        control_switch_position = control_switch_positions.get(((ret[4] >> 8) & 0x40) >> 4)
        more = f', GENSET_STATUS:{genset_status}'
        more += f', CONTROL_SWITCH_POSITION:{control_switch_position}'
        line += more
        cline += more
    else:  # unformatted data
        line += ', data: '
        cline += ', data: '
        for i in ret[4:4 + math.floor(bytes / 2)]:  # ret[3] is in bytes, but values are in words
            line += f'{hex(i >> 8)} '
            line += f'{hex(i & 0x00FF)} '
            cline += f'{hex(i >> 8)} '
            cline += f'{hex(i & 0x00FF)} '
        if bytes % 2 == 1:  # if an odd number, there's one more byte
            line += f'{hex(ret[4 + math.ceil(bytes / 2)] >> 8)} '
            cline += f'{hex(ret[4 + math.ceil(bytes / 2)] >> 8)} '

    #    line += (f', raw: {ret}\n')
    #    cline += (f', raw: {ret}')
    line += '\n'
    window['-LOG-'].print(cline)
    if log_handle is not None:
        log_handle.write(line)
        write_count = write_count + 1
        if write_count >= 100:
            log_handle.flush()
            write_count = 0


# returns the value read or an exception
def read_register(
        registeraddress, number_of_decimals=0, functioncode=3, signed=False
):
    """Read a register from the Modbus server"""
    global comm_status
    try:
        return instrument.read_register(registeraddress, number_of_decimals)  # Registernumber, number of decimals
    except minimalmodbus.IllegalRequestError as e:
        comm_status = str(e)
        raise e
        # sys.exit(6)  # no such device or address
    except minimalmodbus.InvalidResponseError as e:
        comm_status = str(e)
        print(f'Invalid Response error {e}. Continuing...')
    #        sys.exit(74)  # EBADMSG bad message
    except minimalmodbus.NoResponseError as e:
        comm_status = str(e)
        print(comm_status)
        print("[ignoring error]")
        # sys.exit(110) # Connection timed out


def read_registers(addr, val):
    global comm_status
    try:
        return instrument.read_registers(addr, val)  # starting address, number of registers
    except minimalmodbus.IllegalRequestError as e:
        comm_status = str(e)
        print(e)
        # sys.exit(6)  # no such device or address
    except minimalmodbus.InvalidResponseError as e:
        comm_status = str(e)
        print(e)
        return None  # sys.exit(74)  # EBADMSG bad message
    except minimalmodbus.NoResponseError as e:
        comm_status = str(e)
        print(e)
        print('[Ignoring error]')
    except NameError as e:
        raise e
        # print(e)
        # sg.popup('Please connect the corect cable')


#        sys.exit(110)  # Connection timed out linux error code https://kdave.github.io/errno.h/

def write_register(addr, val, desc):
    global comm_status
    try:
        instrument.write_register(addr, val, 0, products[product][4])
    except minimalmodbus.IllegalRequestError as e:
        ret = "Illegal request error"
        comm_status = ret
    except minimalmodbus.InvalidResponseError as e:
        ret = "Invalid response error"
        comm_status = ret
    except minimalmodbus.NoResponseError as e:
        ret = "No communication with the instrument (null answer)"
        comm_status = ret
    else:
        ret = 'OK'
        comm_status = ''
    return ret


def run_script(script):
    ret = ''
    file1 = open(script, 'r')
    lines = [line for line in file1.readlines() if
             len(line.strip()) > 0 and line[0] != '#']  # strip non-executable rows
    rdr = csv.reader(lines, skipinitialspace=True, strict=False)  # parse into list
    # rebuild, converting numeric strings to numbers, and register names to register numbers
    # script_lines=[]
    for row in rdr:
        window['-OUTPUT-'].update(ret)
        window['-COMMS-'].update(comm_status)
        window.refresh()  # at the top because we don't always make it to the bottom of the loop
        if len(row) < 5:
            ret += f'not enough columns in "{row}"\n'
            continue
        b = []

        for a in row: # convert things that look like numbers to numbers
            try:
                b.append(int(a))
            except ValueError as e:
                try:
                    b.append(float(a))
                except ValueError as e:
                    if a.lower() == 'false':
                        b.append(False)
                    elif a.lower() == 'true':
                        b.append(True)
                    else:
                        b.append(a)
        # ret += (str(b) + ":")
        ret = do_script_line(b, ret)
        if len(ret) == 0 or ret[len(ret) - 1] != '\n':
            ret += '\n'
    ret += f'{len(lines)} commands processed.\n'
    if '*** FAIL ***' in ret or '*** ERROR ***' in ret:
        ret += '*** AT LEAST ONE COMMAND FAILED ***'
    else:
        ret += 'No errors detected'

    window['-OUTPUT-'].update(ret)
    window['-COMMS-'].update(comm_status)
    window.refresh()


def is_run_ok(values):
    # check for complete command
    run_ok = True
    if values['-REG-'] == '':
        run_ok = False
    try:
        # converting to integer
        if values['-RADIO0-'] == True:  # int for Read
            if int(values['-VALUE-']) < 0:
                run_ok = False
        else:
            float(values['-VALUE-'])  # float for Write
    except ValueError:
        run_ok = False
    try:  # converting to integer
        if float(values['-DELAY-']) < 0:
            run_ok = False
    except ValueError:
        run_ok = False
    try:  # converting to integer
        int(values['-REPEAT-'])
    except ValueError:
        run_ok = False
    return run_ok


# returns instrument object if connected, None otherwise
def connect_port():
    global instrument, comm_status
    # looking for com port with a CP210x chipset plugged in.
    found_port = ""
    ret = ""
    ports = serial.tools.list_ports.comports()
    baud = products[product][1]
    address = products[product][2]
    if len(ports) == 0:
        comm_status = "No serial port found"
        proceed = sg.popup_ok(comm_status)
        return None

    for port, desc, hwid in sorted(ports):
        ret += "Checking {}: {} [{}]".format(port, desc, hwid)
        if "CP210x" in desc:
            found_port = port
            comm_status = "Found ENG-00000480 cable at " + found_port
            ret += (comm_status)
        if "USB Serial Port" in desc:
            found_port = port
            comm_status = "Found generic USB serial at " + found_port
            ret += (comm_status)
        ret += '\n'

    if found_port == "":
        comm_status = 'ENG-00000480 cable not found'
        ret += comm_status + ". Make sure it's plugged in.\nContinue?"
        proceed = sg.popup_ok("Adapter not found", ret)
        return None
    else:
        # initialize modbus
        try:
            instrument.serial.close() # it can't seem to re-set the baud rate w/o closing first
        except:
            pass
        try:
            comm_status = f'Connecting to {found_port} at {baud} baud and RS485 address {address}... '
            print(comm_status)

            instrument = minimalmodbus.Instrument(found_port, address, baudrate=baud, debug=False,
                                                  timeout=1, close_port_after_each_call=False)  # port name, slave address (in decimal)
            instrument.debug = True  #FIXME DEBUG
            instrument.serial.baudrate = baud  # it can't seem to set the baud rate after the first instatiation
        except serial.serialutil.SerialException as e:
            comm_status = str(e)
            sg.popup_ok(e)
            return False  # sys.exit(2)  # ENOENT, no such file or directory
        return instrument


# MAIN
def main():
    global window, instrument, log_interval, log_handle, products, product
    instrument = connect_port()
    connected = (instrument != None)
    # sg.theme('LightBrown4')

    product_names = [p[0] for p in products]
    # initialize registers
    regs = products[product][3]
    for key in regs:
        reg_vals[key] = None

    # Set up GUI
    reg_names = sorted([reg for reg in regs])
    # reg_names = sorted([reg for reg in regs])
    value_text = ['# of Registers', 'Value']
    # print(reg_names.sort())
    top = [
        [sg.T("Product"),
         sg.Combo(product_names, default_value=products[product][0], key='-PRODUCT-', enable_events=True),
         sg.B("Connect", key='-CONNECT-', disabled=not connected),
         sg.B("CAN", key='-CAN-', disabled=False),
         sg.T(comm_status, key='-COMMS-')],
        [sg.T("Select Modbus Script"),
         sg.Combo(sg.user_settings_get_entry('-filenames-', []),
                  default_value=sg.user_settings_get_entry('-last filename-', ''), size=(130, 1), key='-FILENAME-'),
         sg.FileBrowse(file_types=(('scripts', '*.modbus'),))],
        [sg.Button('Run Script')],
    ]

    col1 = [
        [sg.T('Command Entry'), sg.Push(),
         sg.Combo(reg_names, default_value=reg_names[0], size=30, key='-REG-', enable_events=True)],
        [sg.Push(), sg.Radio('Read', 'RADIO1', key='-RADIO0-', default=True, enable_events=True),
         sg.Radio('Write', 'RADIO1', key="-RADIO1-", default=False, enable_events=True)],
        [sg.Text(value_text[0], key='-VALUE_TEXT-'), sg.Push(),
         sg.Input('1', key='-VALUE-', enable_events=True, size=20)],
        [sg.Text('Delay after'), sg.Push(), sg.Input('0.1', key='-DELAY-', size=20, enable_events=True)],
        [sg.Text('Repetitions'), sg.Push(), sg.Input('1', key='-REPEAT-', size=20, enable_events=True)],
        [sg.B("Run", key='-RUN-', disabled=True)],
        [sg.T('Remote Monitor datalog'),
         sg.Checkbox("Log Enable", default=False, key='-LOGGING-', enable_events=True),
         sg.CBox("Timestamp", default=False, key='-TIMESTAMP-'),
         sg.InputText(key='-FILE-', do_not_clear=False, enable_events=True, visible=False),
         sg.FileSaveAs(file_types=(("CSV", "*.csv"), ("All Files", "*.*")))],
        [sg.Text('Max log interval (ms)'), sg.Push(), sg.Input('80', key='-LOG_INTERVAL-', size=5, enable_events=True),
         sg.Text('Log read delay'), sg.Input(wait, key='-WAIT-', size=6, disabled=True)],
        [sg.Multiline("", key='-LOG-', disabled=True, size=(55, 18), autoscroll=True)],
        [sg.T('See 99-0000020 HyPR 6000 Remote Monitor API.docx for help with values')],
    ]
    col2 = [sg.Text('Results')], [sg.Multiline('', key='-OUTPUT-', size=(92, 30), disabled=True, autoscroll=True)]
    layout = [top, [sg.vtop(sg.Column(col1)), sg.Column(col2)]],
    #    [sg.Text("", size=(160,25), key='-LOG-')],

    window = sg.Window(f'Remote Client (93-0000020, {__version__})', layout)
    # FIXME we need to run this before we return control to the user: window['-RUN-'].update(disabled=not (is_run_ok(values)))

    # start the data logger thread TODO
    # log_thread = threading.Thread(target=readlog, daemon=True)
    # log_thread.start()

    while True:
        event, values = window.read(timeout=log_interval)

        # roughly in order of frequency of use
        if event == '__TIMEOUT__':
            readlog(values, connected, log_handle)
            # showlogs = True   # tell readlog to display what it has
            # window['-WAIT-'].update(wait)
        elif event == '-RADIO0-':
            window['-VALUE_TEXT-'].update(value_text[0])
        elif event == '-RADIO1-':
            window['-VALUE_TEXT-'].update(value_text[1])
        elif event == '-VALUE-' and values['-VALUE-']:
            if values['-RADIO0-'] and values['-VALUE-'][-1] not in '0123456789':  # positive integers for read
                window['-VALUE-'].update(values['-VALUE-'][:-1])
            elif values['-VALUE-'][-1] not in '0123456789.-':  # float for write
                window['-VALUE-'].update(values['-VALUE-'][:-1])
        elif event == '-DELAY-' and values['-DELAY-']:
            if values['-DELAY-'][-1] not in '0123456789.':  # positive float
                window['-DELAY-'].update(values['-DELAY-'][:-1])
        elif event == '-REPEAT-' and values['-REPEAT-']:
            if values['-REPEAT-'][-1] not in '0123456789':  # positive integers
                window['-DELAY-'].update(values['-DELAY-'][:-1])
        elif event == '-LOG_INTERVAL-' and values['-LOG_INTERVAL-']:
            try:
                val = int(values['-LOG_INTERVAL-'])
            except ValueError as e:
                continue
            else:
                if val < 0:
                    val = 0
                elif val > 1000:
                    val = 1000
                log_interval = val
                window['-LOG_INTERVAL-'].update(log_interval)
                # print(f'log_interval:{log_interval}')
        elif event == '-RUN-':
            window['-RUN-'].update(disabled=True)
            do_script_line(
                [values['-REG-'], int(values['-VALUE-']), 0 if values['-RADIO0-'] else 1, float(values['-DELAY-']),
                 int(values['-REPEAT-']), ''], '')
            window['-RUN-'].update(disabled=False)
            window['-COMMS-'].update(comm_status)
        elif event == 'Run Script':
            sg.user_settings_set_entry('-filenames-', list(
                set(sg.user_settings_get_entry('-filenames-', []) + [values['-FILENAME-'], ])))
            sg.user_settings_set_entry('-last filename-', values['-FILENAME-'])
            run_script(values['-FILENAME-'])
            window['-COMMS-'].update(comm_status)
        elif event == '-PRODUCT-':
            product = product_names.index(values['-PRODUCT-'])
            regs = products[product][3]
            reg_names = sorted([reg for reg in regs])
            window['-REG-'].update(value=reg_names[0], values=reg_names)
            instrument = connect_port()
            connected = (instrument != None)
            window['-COMMS-'].update(comm_status)
        elif event == '-FILE-':
            filename = values['-FILE-']
            if filename:
                log_handle = open(filename, 'w')

        elif event == '-CONNECT-':
            instrument = connect_port()
            connected = (instrument != None)
            window['-COMMS-'].update(comm_status)
        elif event == '-CAN-':
            sg.popup("CAN feature disabled")
            # if (can_connect()):
            #     ani = FuncAnimation(plt.gcf(), animate, 10)
            #     plt.tight_layout()
            #     plt.show()
        elif event == sg.WINDOW_CLOSED or event == 'Quit':
            break

        else:
            pass

        window['-CONNECT-'].update(disabled=connected)
        window['-RUN-'].update(disabled=(not (is_run_ok(values)) or not connected))

    # cleanup thread(s)
    # if log_thread.is_alive():
    #     log_thread.join()
    # Finish up by removing from the screen
    if log_handle is not None:
        log_handle.close()
    window.close()
    print("Main    : wait for the thread to finish")
    # x.join()
    print("Main    : all done")

    # def log():
    #     # just sit back and data log
    #     print("Echoing data log. Cancel witih CTRL-C\n\n")
    #     while True:
    #         try:
    #             time.sleep(minimum_delay)
    #             ret = read_registers(regs["LOG_LEVEL"], 8)
    #             # handling logging as a special case, because we don't care to see a bunch of empty reads
    #
    #             if ret is not None and ret[0] != 0x0F:  # if we got a bad read or log is empty, do nothing, else print it and clear it
    #                 # if True and ret is not None:
    #                 showlog(ret, False, f)
    #                 time.sleep(normal_delay)
    #             # the following line is for debugging the log circular buffer
    #             # do_script_line(
    #             #         [regs["LOG_HEAD"], 3, False, normal_delay, 1, 'should return log head, tail and status'])
    #         except (KeyboardInterrupt) as e:
    #             print("Sorry to see you go\n")
    #             sys.exit(0)
    #
    #         except (minimalmodbus.NoResponseError) as e:
    #             print(e)


if __name__ == '__main__':
    main()
