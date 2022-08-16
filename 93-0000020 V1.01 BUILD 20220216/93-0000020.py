#!/usr/bin/env python3
# reference implementation for the Solar Stik Modbustik protocol.


__author__ = "Brian Alano"
__license__ = "(C) 2020-2022 Solar Stik, Inc. All rights reserved."
__status__ = "Development"
__url__ = "https://www.solarstik.com"
__version__ = "V1.01 BUILD 202202016"

#####################################################################################
# Configuration parameters
port = 'COM5'
baud = 256000
debug = False
address = 1

import sys
import PySimpleGUI as sg
import time
import minimalmodbus
import math
import serial.tools.list_ports  # from pyserial
import csv

# must match register list in RMAGS.h
regs = {
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
    "GENERATOR_POWER_SETTINGS": 53, # watts. 1000 to 5000 watts by 100s
    "AVAILABLE_CURRENT": 54, # in A. available current given available power
    "CHARGER_MODE": 55, # 0 - CC, 1 - CV, 2 - Off, 3 - Delay
    "TOTAL_REGS_SIZE": 56
    # a hack to tell us the total number of registers. Function 3 and 16 share the same register array
}
BIT_EXECUTE = 0x0001
LOG_ENTRY_SIZE = 8
log_levels = ["EMERG", "ALERT", "CRIT", "ERR", "WARNING", "NOTICE", "INFO", "DEBUG",
              "UNDEF", "UNDEF", "UNDEF", "UNDEF", "UNDEF", "UNDEF", "UNDEF", "NULL"]
devices = ['CAN_A', 'CAN_B', 'RS485']
codes = {0: 'null',
         1: 'ADDRESS_CLAIMED',
         2: 'J1939_STATUS',
         0xFF17: 'AdvDCS Response',
         0xFF15: 'Controller 1st PGN',
         0xFF19: 'Controller 2nd PGN',
         0xFF22: 'Controller 3rd PGN'
         }

j1939_statuses = {
    0: 'ADDRESSCLAIM_INIT',
    1: 'ADDRESSCLAIM_INPROGRESS',
    2: 'NORMALDATATRAFFIC',
    3: 'ADDRESSCLAIM_FAILED'
}

normal_delay = 0.0  # .03 s seems slow enough. .02 s is too fast
minimum_delay = 0.01

#####################################################################################

#####################################################################################
# Coding conventions
# Error codes from linux error code https://kdave.github.io/errno.h/, just because
#####################################################################################

#
# line - command to execute, as a list of parameters
# preserve - content of -OUTPUT- to not overwrite during an update to -OUTPUT-
def do_script_line(line, preserve):
    """Process a line from the script"""
    ret=preserve
    #ret+=(f'{line}: ')
    try:
        addr = regs[line[0]]
    except (KeyError) as e:
        ret = f'*** ERROR *** unknown key {line[0]}'
    else:
        val = line[1]
        write = False if len(line) < 3 else line[2]
        delay = 0 if len(line) < 4 else line[3]
        iterations = 1 if len(line) < 5 else line[4]
        desc = '' if len(line) < 6 or len(line[5].strip()) == 0 else line[5].strip()
        if len(desc)>=7 and desc[0:7].lower() == "assert:":
            assertion=desc[7:].strip()
        else:
            assertion=''
        desc = ': ' if len(desc) == 0 else f' [{desc}]: '
        # print(f'{line[5]}, {desc}, {assertion}')
        if write:
            ret += (f'Write {val} to {list(regs.keys())[addr]}({addr})')
        else:
            if val > 1:  # read multiple registers
                ret+=(
                    f'Read {val} registers starting at {list(regs.keys())[addr]}({addr})')
            else:
                ret+=(f'Read from {list(regs.keys())[addr]}({addr})')

        ret += desc

        for i in range(iterations):
            if iterations > 1:
                ret += (f'{i + 1}:')
            if write:
                result =write_register(addr, val, desc)
                ret += result
            else:
                if val > 1:  # read_registers()
                    result = (read_registers(addr, val))
                else:  # read_register
                    result = (read_register(addr, 0))
                ret += str(result)
            if len(assertion)>0:
                try:
                    asserted = eval(assertion, {}, {"x": result})
                except (NameError, TypeError, SyntaxError) as e:
                    ret += f': *** ERROR *** {e}'
                else:
                    if asserted:
                        ret += ': PASS'
                    else:
                        ret += ': *** FAIL ***'
            if i+1 < iterations:
                ret += '; '
            window['-OUTPUT-'].update(ret)
            window.refresh()
            time.sleep(delay)
        ret += '\n'

    window['-OUTPUT-'].update(ret)
    window.refresh()

    return ret

def showlog(ret, show_time):
    print = window['-LOG-'].print

    if show_time:
        print(f'{time.asctime()} ', end='')
    # decode
    if ret[0] > len(log_levels) - 1:
        print(f'{ret[0]}', end='', text_color='red')
    else:
        print(f'{log_levels[ret[0]]}', end='')
    code = codes.get(ret[1])
    if code:
        print(f', {code}', end='')
    else:
        print(f", {hex(ret[1])}", end='', text_color='red')
    if ret[2] > len(devices) - 1:
        print(f', illegel device: {ret[2]}', end='', text_color='red')
    else:
        print(f', {devices[ret[2]]}', end='')
    bytes = ret[3]
    if bytes <= 8:
        print(f', bytes:{bytes}', end='')
    else:
        print(f', bytes(illegal value, max 8):{bytes}', end='', text_color='red')
        bytes = 8  # show the legal number
    if code == 'J1939_STATUS':
        j1939_status = j1939_statuses.get(ret[4])
        if j1939_status:
            print(f', {j1939_status}', end='')
        else:
            print(f", {hex(j1939_status)}", end='', text_color='red')
    elif code == 'Controller 1st PGN':
        print(f', GEN_CONTROL:{ret[4] & 0x00FF}', end='')
        print(f', max charge current:{ret[5] & 0x00FF}', end='')
        print(f', start delay:{ret[6] >> 8}', end='')
        print(f', stop delay:{ret[6] & 0x00FF}', end='')
        print(f', LVCO:{ret[7]}', end='')
    else:  # unformatted data
        print(', data: ', end='')
        for i in ret[4:4 + math.floor(bytes / 2)]:  # ret[3] is in bytes, but values are in words
            print(f'{hex(i >> 8)} ', end='')
            print(f'{hex(i & 0x00FF)} ', end='')
        if (bytes % 2 == 1):  # if an odd number, there's one more byte
            print(f'{hex(ret[4 + math.ceil(bytes / 2)] >> 8)} ', end='')

    # terminate line
    print(f', raw: {ret}')


def read_register(
        registeraddress, number_of_decimals=0, functioncode=3, signed=False
):
    """Read a register from the Modbus server"""

    try:
        return instrument.read_register(registeraddress, number_of_decimals)  # Registernumber, number of decimals
    except (minimalmodbus.IllegalRequestError) as e:
        raise e
        # sys.exit(6)  # no such device or address
    except (minimalmodbus.InvalidResponseError) as e:
        print(e)
        sys.exit(74)  # EBADMSG bad message
    except (minimalmodbus.NoResponseError) as e:
        print(e)
        print("[ignoring error]")
        # sys.exit(110) # Connection timed out

def read_registers(addr, val):
    try:
        return instrument.read_registers(addr, val)  # starting address, number of registers
    except (minimalmodbus.IllegalRequestError) as e:
        print(e)
        # sys.exit(6)  # no such device or address
    except (minimalmodbus.InvalidResponseError) as e:
        print(e)
        return None  # sys.exit(74)  # EBADMSG bad message
    except (minimalmodbus.NoResponseError) as e:
        print(e)
        print('[Ignoring error]')
    except NameError as e:
        print(e)
        sg.popup('Please connect the corect cable')
        sys.exit(-1)

#        sys.exit(110)  # Connection timed out linux error code https://kdave.github.io/errno.h/

def write_register(addr, val, desc):
    try:
        instrument.write_register(addr, val, 0)
    except (minimalmodbus.IllegalRequestError) as e:
        ret = "Illegal request error"
    except (minimalmodbus.InvalidResponseError) as e:
        ret = "Invalid response error"
    else:
        ret = ('OK')
    return ret

def run_script(script):
    ret = ''
    file1 = open(script, 'r')
    lines = [line for line in file1.readlines() if len(line.strip()) > 0 and line[0] != '#'] # strip non-executable rows
    rdr = csv.reader(lines, skipinitialspace=True, strict=False) # parse into list
    # rebuild, converting numeric strings to numbers, and register names to register numbers
    #script_lines=[]
    for row in rdr:
        window['-OUTPUT-'].update(ret)
        window.refresh() # at the top because we don't always make it to the bottom of the loop
        if len(row) < 5:
            ret += f'not enough columns in "{row}"\n'
            continue
        b=[]

        for a in row:
            try:
                b.append(int(a))
            except (ValueError) as e:
                try:
                    b.append(float(a))
                except (ValueError) as e:
                    if a.lower() == 'false':
                        b.append(False)
                    elif a.lower()== 'true':
                        b.append(True)
                    else:
                        b.append(a)
        #ret += (str(b) + ":")
        ret = do_script_line(b, ret)
        if ret[len(ret)-1] != '\n':
            ret += '\n'
    ret += f'{len(lines)} commands processed.\n'
    if '*** FAIL ***' in ret or '*** ERROR ***' in ret:
        ret += '*** AT LEAST ONE COMMAND FAILED ***'
    else:
        ret += 'No errors detected'

    window['-OUTPUT-'].update(ret)
    window.refresh()

def is_run_ok(values):
    #check for complete command
    run_ok = True
    if values['-REG-'] == '':
        run_ok = False
    try:
        # converting to integer
        if values['-RADIO0-'] == True: # int for Read
            if int(values['-VALUE-']) < 0:
                run_ok = False
        else:
            float(values['-VALUE-']) # float for Write
    except ValueError:
        run_ok = False
    try:         # converting to integer
        if float(values['-DELAY-']) < 0:
            run_ok = False
    except ValueError:
        run_ok = False
    try:         # converting to integer
        int(values['-REPEAT-'])
    except ValueError:
        run_ok = False
    return run_ok

# MAIN
# looking for com port with a CP210x chipset plugged in.
found_port = ""
ret = ""

#sg.theme('LightBrown4')

ports = serial.tools.list_ports.comports()

for port, desc, hwid in sorted(ports):
    ret += "Checking {}: {} [{}]".format(port, desc, hwid)
    if "CP210x" in desc:
        found_port = port
        ret += ("Found ENG-00000480 cable at " + found_port)
    ret += '\n'

if found_port == "":
    ret += ("ENG-00000480 cable not found. Make sure it's plugged in.\nContinue?")
    proceed = sg.popup_ok_cancel("Adapter not found", ret)
    if proceed == "Cancel":
        sys.exit(2)
else:
    # initialize modbus
    try:
        print(f'Connecting to {found_port} at {baud} baud and RS485 address {address}... ')
        instrument = minimalmodbus.Instrument(port, address, baudrate=baud, debug=debug,
                                              timeout=1)  # port name, slave address (in decimal)
    except serial.serialutil.SerialException as e:
        print(e)
        sys.exit(2)  # ENOENT, no such file or directory

# Set up GUI
reg_names = sorted([reg for reg in regs if not reg.startswith("LOG")])
value_text = ['# of Registers', 'Value']
# print(reg_names.sort())
top = [
        [sg.T("Select Modbus Script"), sg.Combo(sg.user_settings_get_entry('-filenames-', []),
          default_value=sg.user_settings_get_entry('-last filename-', ''), size=(130, 1), key='-FILENAME-'),
 sg.FileBrowse(file_types=(('scripts', '*.modbus'),))],
[sg.Button('Run Script')],
       ]

col1 = [
        [sg.T('Command Entry'), sg.Push(), sg.Combo(reg_names, default_value='BUS_VOLTAGE', size=30, key='-REG-', enable_events=True)],
         [sg.Push(), sg.Radio('Read', 'RADIO1', key='-RADIO0-', default=True, enable_events=True),
          sg.Radio('Write', 'RADIO1', key="-RADIO1-", default=False, enable_events=True)],
         [sg.Text(value_text[0], key='-VALUE_TEXT-'), sg.Push(),
          sg.Input('1', key='-VALUE-', enable_events=True, size=20)],
         [sg.Text('Delay after'), sg.Push(), sg.Input('0.1', key='-DELAY-', size=20, enable_events=True)],
         [sg.Text('Repetitions'), sg.Push(), sg.Input('1', key='-REPEAT-', size=20, enable_events=True)],
         [sg.B("Run", key='-RUN-', disabled=False)],
        [sg.T('Remote Monitor datalog'), sg.Checkbox("Log Enable", default = False, key='-LOGGING-', enable_events=True), sg.CBox("Timestamp", default=False, key='-TIMESTAMP-') ],
        [sg.Multiline("", key='-LOG-', disabled=True, size=(55, 17.5), autoscroll=True)],
        [sg.T('See 99-0000020 HyPR 6000 Remote Monitor API.docx for help with values')],
    ]
col2 = [sg.Text('Results')], [sg.Multiline('', key='-OUTPUT-', size=(92,30), disabled=True, autoscroll=True) ]
layout = [top, [sg.vtop(sg.Column(col1)), sg.Column(col2)]],
#    [sg.Text("", size=(160,25), key='-LOG-')],

window = sg.Window(f'HyPR 6000 Client (93-0000020, {__version__})', layout)
#FIXME we need to run this before we return control to the user: window['-RUN-'].update(disabled=not (is_run_ok(values)))

while True:
    event, values = window.read(timeout = 100)

    if event == '__TIMEOUT__':
        ret = read_registers(regs["LOG_LEVEL"], 8)
        # handling logging as a special case, because we don't care to see a bunch of empty reads

        if ret is not None and ret[0] != 0x0F and values['-LOGGING-'] == True:  # if we got a bad read or log is empty, do nothing, else print it and clear it
            # if True and ret is not None:
            showlog(ret, values['-TIMESTAMP-'])

    elif event == '-RADIO0-':
        window['-VALUE_TEXT-'].update(value_text[0])
    elif event == '-RADIO1-':
        window['-VALUE_TEXT-'].update(value_text[1])
    elif event == '-VALUE-' and values['-VALUE-']:
        if values['-RADIO0-'] and values['-VALUE-'][-1] not in ('0123456789'):  # positive integers for read
            window['-VALUE-'].update(values['-VALUE-'][:-1])
        elif values['-VALUE-'][-1] not in ('0123456789.-'):  # float for write
            window['-VALUE-'].update(values['-VALUE-'][:-1])
    elif event == '-DELAY-' and values['-DELAY-']:
        if values['-DELAY-'][-1] not in ('0123456789.'):  # positive float
            window['-DELAY-'].update(values['-DELAY-'][:-1])
    elif event == '-REPEAT-' and values['-REPEAT-']:
        if values['-REPEAT-'][-1] not in ('0123456789'):  # positive integers
            window['-DELAY-'].update(values['-DELAY-'][:-1])
    elif event == '-RUN-':
        window['-RUN-'].update(disabled=True)
        do_script_line([values['-REG-'], int(values['-VALUE-']), 0 if values['-RADIO0-'] else 1, float(values['-DELAY-']), int(values['-REPEAT-']), ''], '')
        window['-RUN-'].update(disabled=False)
    elif event == 'Run Script':
            sg.user_settings_set_entry('-filenames-', list(
                set(sg.user_settings_get_entry('-filenames-', []) + [values['-FILENAME-'], ])))
            sg.user_settings_set_entry('-last filename-', values['-FILENAME-'])
            run_script(values['-FILENAME-'])
    elif event == sg.WINDOW_CLOSED or event == 'Quit':
        break

    else:
            pass

    window['-RUN-'].update(disabled=not(is_run_ok(values)))


# Finish up by removing from the screen
window.close()


def log():
    # just sit back and data log
    print("Echoing data log. Cancel witih CTRL-C\n\n")
    while True:
        try:
            time.sleep(minimum_delay)
            ret = read_registers(regs["LOG_LEVEL"], 8)
            # handling logging as a special case, because we don't care to see a bunch of empty reads

            if ret is not None and ret[
                0] != 0x0F:  # if we got a bad read or log is empty, do nothing, else print it and clear it
                # if True and ret is not None:
                showlog(ret)
                time.sleep(normal_delay)
            # the following line is for debugging the log circular buffer
            # do_script_line(
            #         [regs["LOG_HEAD"], 3, False, normal_delay, 1, 'should return log head, tail and status'])
        except (KeyboardInterrupt) as e:
            print("Sorry to see you go\n")
            sys.exit(0)

        except (minimalmodbus.NoResponseError) as e:
            print(e)
