#!/usr/bin/python3
# -*- coding: utf-8 -*-
# MIT License
# 
# Copyright (c) 2019 Jean-Manuel CABA
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Based on numerous project that are almost similar in codes
# see: https://github.com/sandalian/bluefence/blob/master/bluefence.py
#      https://github.com/FrederikBolding/bluetooth-proximity
#      https://github.com/ewenchou/bluetooth-proximity/blob/master/bt_proximity/bt_rssi.py
#      https://github.com/dagar/bluetooth-proximity/blob/master/proximity_dagar.py
#      https://github.com/ewenchou/bluetooth-proximity/blob/master/examples/lnsm/lnsm.py
# intended as a replacement for Blueproximity
# for RSSI to distance
# read papers and information:
#      https://www.cise.ufl.edu/~qathrady/reports/BLE.pdf and
#      https://www.rn.inf.tu-dresden.de/dargie/papers/icwcuca.pdf
#      https://iotandelectronics.wordpress.com/2016/10/07/how-to-calculate-distance-from-the-rssi-value-of-the-ble-beacon/
#      https://stackoverflow.com/questions/36399927/distance-calculation-from-rssi-ble-android
#      here https://python-forum.io/Thread-Python-Ble-Distance-Problem

import time
import bluetooth
# noinspection PyProtectedMember
import bluetooth._bluetooth as bt
import struct
import array
import fcntl


class BleRssi(object):
    def __init__(self, addr):
        self._addr = addr
        self._hci_sock = bt.hci_open_dev()
        self._hci_fd = self._hci_sock.fileno()
        self._bt_sock = bluetooth.BluetoothSocket(bluetooth.L2CAP)
        self._bt_sock.settimeout(10)
        self._connected = False
        self._cmd_pkt = None

    def _prep_cmd_pkt(self):
        _request_command = struct.pack(b'6sB17s', bt.str2ba(self._addr), bt.ACL_LINK, b'\0' * 17)
        _request_as_array = array.array('b', _request_command)
        _ = fcntl.ioctl(self._hci_fd, bt.HCIGETCONNINFO, _request_as_array, 1)
        _handle = struct.unpack(b'8xH14x', _request_as_array.tobytes())[0]
        self._cmd_pkt = struct.pack('H', _handle)

    def connect(self):
        # noinspection PyUnresolvedReferences
        self._bt_sock.connect_ex((self._addr, 1))
        self._connected = True

    # see code of https://github.com/r10r/bluez/blob/master/lib/hci.c
    def get_rssi(self):
        try:
            # noinspection PyPep8Naming
            READ_RSSI_RP_SIZE = 4
            if not self._connected:
                self.connect()
            if self._cmd_pkt is None:
                self._prep_cmd_pkt()
            _rssi = bt.hci_send_req(self._hci_sock,
                                    bt.OGF_STATUS_PARAM,
                                    bt.OCF_READ_RSSI,
                                    bt.EVT_CMD_COMPLETE,
                                    READ_RSSI_RP_SIZE,
                                    self._cmd_pkt)
            _rssi = struct.unpack('b', _rssi[3].to_bytes(1, 'big'))
            _rssi = _rssi[0]
            if _rssi is not None:
                return float(_rssi)
            self._connected = False
            return None
        except IOError:
            self._connected = False
            return None


class BleTxPower(object):
    def __init__(self, addr):
        self._addr = addr
        self._hci_sock = bt.hci_open_dev()
        self._hci_fd = self._hci_sock.fileno()
        self._bt_sock = bluetooth.BluetoothSocket(bluetooth.L2CAP)
        self._bt_sock.settimeout(10)
        self._connected = False
        self._cmd_pkt = None

    def _prep_cmd_pkt(self):
        _request_command = struct.pack(b'6sB17s', bt.str2ba(self._addr), bt.ACL_LINK, b'\0' * 17)
        _request_as_array = array.array('b', _request_command)
        fcntl.ioctl(self._hci_fd, bt.HCIGETCONNINFO, _request_as_array, 1)
        _handle = struct.unpack(b'8xH14x', _request_as_array.tobytes())[0]
        self._cmd_pkt = struct.pack('H', _handle)

    def connect(self):
        # noinspection PyUnresolvedReferences
        self._bt_sock.connect_ex((self._addr, 1))
        self._connected = True

    # see code of https://github.com/r10r/bluez/blob/master/lib/hci.c
    # noinspection PyPep8Naming
    def get_tx_power(self):
        try:
            # noinspection PyPep8Naming
            READ_TRANSMIT_POWER_LEVEL_RP_SIZE = 4
            if not self._connected:
                self.connect()
            if self._cmd_pkt is None:
                self._prep_cmd_pkt()
            _tx_power = bt.hci_send_req(self._hci_sock,
                                        bt.OGF_HOST_CTL,
                                        bt.OCF_READ_TRANSMIT_POWER_LEVEL,
                                        bt.EVT_CMD_COMPLETE,
                                        READ_TRANSMIT_POWER_LEVEL_RP_SIZE,
                                        self._cmd_pkt)
            _tx_power = struct.unpack('b', _tx_power[3].to_bytes(1, 'big'))[0]
            if _tx_power is not None:
                return float(_tx_power)
            self._connected = False
            return None
        except IOError:
            self._connected = False
            return None


def get_rssi(bt_addr):
    _b = BleRssi(addr=bt_addr)
    _rssi = _b.get_rssi()
    return _rssi


def get_tx_power(bt_addr):
    _b = BleTxPower(addr=bt_addr)
    _tx_power = _b.get_tx_power()
    return _tx_power


# (tuned for my phone ... may need tuning for other phones)
MY_PHONE_RSSI_AT_1m = -4.0


def rssi_to_distance(input_rssi):
    import math

    n = 2.5  # Path loss exponent(n) = Range from 2 to 4
    measured_power = MY_PHONE_RSSI_AT_1m  # aka RSSI value at 1 meter

    x = float((measured_power - float(input_rssi)) / (10.0 * n))
    distance = math.pow(10, x)

    return distance


def old_calculate_accuracy(tx_power, input_rssi):
    import math
    # noinspection PyBroadException,PyPep8
    try:
        ratio = float(input_rssi) / float(tx_power)
        return 0.89976 * math.pow(ratio, 7.7095) + 0.111
    except:
        return rssi_to_distance(input_rssi)


def new_calculate_accuracy(tx_power, input_rssi):
    import math
    # noinspection PyBroadException,PyPep8
    try:
        ratio = float(input_rssi) / float(tx_power)
        return 0.42093 * math.pow(ratio, 6.9476) + 0.54992
    except:
        return rssi_to_distance(input_rssi)


def calculate_accuracy(tx_power, input_rssi):
    d1 = float(old_calculate_accuracy(tx_power, input_rssi))
    d2 = float(new_calculate_accuracy(tx_power, input_rssi))
    return (d1 + d2) / 2.0


# in case no measurement ... default to 1 meter values
# if there was not previous measurement
previous_avg_rssi = MY_PHONE_RSSI_AT_1m
previous_avg_distance = 1.0
previous_tx_power = MY_PHONE_RSSI_AT_1m


def get_distance_approximation(bt_addr):
    global previous_avg_rssi
    global previous_avg_distance
    global previous_tx_power

    # this numbers are totally arbitrary
    num_loop = 97
    loop_step_sleep = 0.01

    all_rssi = []
    all_distances = []

    for i in range(1, num_loop):
        _rssi_bt = get_rssi(bt_addr)
        if _rssi_bt is None:
            continue
        _rssi_bt = float(_rssi_bt)

        distance1 = rssi_to_distance(_rssi_bt)

        _tx_power_bt = get_tx_power(bt_addr)
        if _tx_power_bt is None:
            _tx_power_bt = previous_tx_power

        previous_tx_power = _tx_power_bt

        distance2 = calculate_accuracy(_tx_power_bt, _rssi_bt)
        if distance2 > 1.0:
            distance = distance1
        else:
            # use distance2 to tune approximation
            distance = (distance1 * 3 + distance2) / 4.0

        all_rssi.append(_rssi_bt)
        all_distances.append(distance)

        time.sleep(loop_step_sleep)

    assert len(all_rssi) == len(all_distances)

    if len(all_rssi) == 0:
        avg_rssi = previous_avg_rssi
        avg_distance = previous_avg_distance
    else:
        avg_rssi = sum(all_rssi) / len(all_rssi)
        avg_distance = sum(all_distances) / len(all_distances)

    previous_avg_rssi = avg_rssi
    previous_avg_distance = avg_distance

    return avg_distance


VERBOSE = True
INFO = True


def _verbose_log(msg):
    if VERBOSE:
        print(msg)


def _main():
    import sys
    import os

    # time between searches for device in seconds
    scan_period = 1
    # The command to run when the device is out of range
    # noinspection PyPep8
    run_xscreensaver_if_killed = '(xscreensaver-command --time 2>&1|grep -q "no screensaver is running" && xscreensaver &)'
    if_bt_gone = run_xscreensaver_if_killed + '; xscreensaver-command -lock'
    # The command to run when the device is back in range
    # noinspection PyPep8
    if_bt_back = '(xscreensaver-command -time 2>&1|grep -q "screen non-blanked since" && xscreensaver-command -deactivate || killall xscreensaver); ' + run_xscreensaver_if_killed

    max_missed = 3

    if len(sys.argv) < 2:
        print("Usage " + sys.argv[0] + " <btaddr>")
        sys.exit(1)

    bt_addr = sys.argv[1]
    bt_in_range = True
    away_counter = 0
    far_counter = 0

    _verbose_log("Identifying device...")

    # noinspection PyPep8,PyBroadException
    try:

        # first scan to ensure bluetooth works
        nearby_devices = bluetooth.discover_devices(duration=8, lookup_names=True, flush_cache=True, lookup_class=False)
        print("found %d devices" % len(nearby_devices))

        print("performing inquiry...")
        # initial check, see if mentioned BT device active. If it's not, clean exit
        # noinspection PyPep8Naming
        btName = bluetooth.lookup_name(bt_addr, timeout=5)

        if btName:
            _verbose_log('OK: Found your device ' + btName)

            while True:
                who = bluetooth.lookup_name(bt_addr, timeout=2)
                if who:
                    assert btName == who
                    bt_in_range = True

                    distance = get_distance_approximation(bt_addr)
                    print("Approximate distance: " + str(distance * 100.0) + " cm")

                    if distance * 100.0 > 150.0:
                        _verbose_log('DEVICE FAR')
                        status = 'far'
                        far_counter += 1

                        if far_counter >= max_missed:
                            _verbose_log('RUNNING GONE COMMAND AFTER THRESHOLD (FAR)')
                            status = 'far_need_lock'
                            os.system(if_bt_gone)
                    else:
                        _verbose_log('DEVICE NEAR')
                        status = 'near'
                        away_counter = 0
                        far_counter = 0
                        os.system(if_bt_back)
                else:
                    _verbose_log('DEVICE AWAY')
                    away_counter += 1
                    status = 'away'

                    if away_counter >= max_missed:
                        _verbose_log('RUNNING GONE COMMAND AFTER THRESHOLD (AWAY)')
                        os.system(if_bt_gone)
                        status = 'away_need_lock'
                        bt_in_range = False
                time.sleep(scan_period)
                print(status,
                      '| far counter=', far_counter,
                      '| away counter', away_counter,
                      '|', bt_in_range, '|', time.strftime('%H:%M:%S'))
        else:
            sys.exit('ERROR: Your bluetooth device is not active')
    # this usually happen when your PC's bluetooth is disabled.
    except:
        import traceback
        traceback.print_exc()
        sys.exit('ERROR: Bluetooth on machine is not active or unknown error')


if __name__ == "__main__":
    _main()
