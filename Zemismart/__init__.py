# Python module for control of Zemismart Roller Shade
#
#

import struct
import threading
import time

from bluepy import btle


class Zemismart(btle.DefaultDelegate):
    class Timer:
        REPEAT_SUNDAY = 0x01
        REPEAT_MONDAY = 0x02
        REPEAT_TUESDAY = 0x04
        REPEAT_WEDNESDAY = 0x08
        REPEAT_THURSDAY = 0x10
        REPEAT_FRIDAY = 0x20
        REPEAT_SATURDAY = 0x40

        REPEAT_EVERY_DAY = REPEAT_SUNDAY | REPEAT_MONDAY | REPEAT_TUESDAY | REPEAT_WEDNESDAY | REPEAT_THURSDAY \
            | REPEAT_FRIDAY | REPEAT_SATURDAY

        def __init__(self, enabled, position, repeats, hours, minutes):
            """
            Parameters:
                enabled (bool): defines whether the timer is enabled or not
                position (int): desired shares position
                repeats (int): day when the timer should be repeated (use `binary or` to create desired combination,
                               e.g. `Zemismart.Timer.REPEAT_SUNDAY | Zemismart.Timer.REPEAT_FRIDAY`)
                hours (int): hour when the timer should trigger
                minutes (int): minute when the timer should trigger
            """
            if type(enabled) is not bool:
                raise AttributeError('`enabled` must be bool')
            self.enabled = enabled

            if type(position) is not int or not (0 <= position <= 100):
                raise AttributeError('`position` must be integer between 0 and 100')
            self.position = position

            if type(repeats) is not int or repeats > self.REPEAT_EVERY_DAY:
                raise AttributeError('`repeats` must be integer less than %d' % self.REPEAT_EVERY_DAY)
            self.repeats = repeats

            if type(hours) is not int or not (0 <= hours <= 23):
                raise AttributeError('`hours` must be integer between 0 and 23')
            self.hours = hours

            if type(minutes) is not int or not (0 <= minutes <= 59):
                raise AttributeError('`minutes` must be integer between 0 and 59')
            self.minutes = minutes

    datahandle_uuid = "fe51"

    response_start_byte = 0x9a

    start_bytes = bytearray.fromhex('00ff00009a')

    pin_cmd = bytearray.fromhex('17')
    move_cmd = bytearray.fromhex('0a')
    set_position_cmd = bytearray.fromhex('0d')

    get_battery_cmd = bytearray.fromhex('a2')
    get_position_cmd = bytearray.fromhex('a7')
    finished_moving_cmd = bytearray.fromhex('a1')
    update_timer_cmd = bytearray.fromhex('15')

    get_timers_cmd = bytearray.fromhex('a8')
    unknown_cmd_a9 = bytearray.fromhex('a9')

    open_data = bytearray.fromhex('dd')
    close_data = bytearray.fromhex('ee')
    stop_data = bytearray.fromhex('cc')

    def __init__(self, mac="02:4E:F0:E8:7F:63", pin=8888, max_connect_time=30, withMutex=False, iface=None):
        self.mac = mac
        self.pin = pin
        self.max_connect_time = max_connect_time
        self.device = None
        self.datahandle = None
        self.battery = 0
        self.position = 0
        self.withMutex = withMutex
        self.last_command_status = None
        self.iface = iface
        self.timers = []
        self._get_position_responses = set()
        if self.withMutex:
            self.mutex = threading.Lock()
        btle.DefaultDelegate.__init__(self)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type=None, exc_val=None, exc_tb=None):
        self.disconnect()

    def disconnect(self):
        try:
            if self.device:
                self.device.disconnect()
                self.device = None
        except Exception as ex:
            print("Could not disconnect with mac: " + self.mac + ", Error: " + str(ex))
        finally:
            if self.withMutex:
                self.mutex.release()

    def connect(self):
        if self.withMutex:
            self.mutex.acquire()

        try:
            self.device = btle.Peripheral()
            self.device.withDelegate(self)

            start_time = time.time()
            connection_try_count = 0

            while True:
                connection_try_count += 1
                if connection_try_count > 1:
                    print("Retrying to connect to device with mac: " +
                          self.mac + ", try number: " + str(connection_try_count))
                try:
                    self.device.connect(self.mac, addrType=btle.ADDR_TYPE_PUBLIC, iface=self.iface)
                    break
                except btle.BTLEException as ex:
                    print("Could not connect to device with mac: " + self.mac + ", error: " + str(ex))
                    if time.time() - start_time >= self.max_connect_time:
                        print("Connection timeout")
                        raise

            handles = self.device.getCharacteristics()
            for handle in handles:
                if handle.uuid == self.datahandle_uuid:
                    self.datahandle = handle

            if self.datahandle is None:
                self.device = None
                raise Exception("Unable to find all handles")

            self.login()
        except:
            self.disconnect()
            raise

    def handleNotification(self, handle, data):
        if handle == self.datahandle.getHandle() and data[0] == self.response_start_byte:
            if data[1] == self.get_battery_cmd[0]:
                battery = data[7]
                self.save_battery(battery)
                self.last_command_status = True
            elif data[1] == self.get_position_cmd[0]:
                pos = data[5]
                self.save_position(pos)
                self._get_position_responses.add(data[1])
                self._check_get_position_response()
            elif data[1] == self.finished_moving_cmd[0]:
                pos = data[4]
                self.save_position(pos)
                self.last_command_status = True
            elif data[1] == self.unknown_cmd_a9[0]:
                self._get_position_responses.add(data[1])
                self._check_get_position_response()
            elif data[1] == self.get_timers_cmd[0]:
                self._get_position_responses.add(data[1])
                self.timers = []
                # Single timer takes 5 bytes
                for offset in range(0, int(int(data[2]) / 5)):
                    timer_data_start = 3 + offset * 5
                    timer = Zemismart.Timer(
                        bool(data[timer_data_start]),
                        int(data[timer_data_start + 1]),
                        int(data[timer_data_start + 2]),
                        int(data[timer_data_start + 3]),
                        int(data[timer_data_start + 4])
                    )
                    self.timers.append(timer)

                self._check_get_position_response()
            elif data[1] in (self.set_position_cmd[0], self.pin_cmd[0], self.move_cmd[0], self.update_timer_cmd[0]):
                if data[3] == 0x5A:
                    self.last_command_status = True
                elif data[3] == 0xA5:
                    self.last_command_status = False

    def _check_get_position_response(self):
        """
        Command 0xA7 (get_position) yields three responses from the device. We can't treat the command result as
        a success, unless we have receieved all of them.
        This method checks that since the last update() call, where self._get_position_responses is reset, we
        receieved all the required responses.
        """
        if self._get_position_responses == {self.get_position_cmd[0], self.get_timers_cmd[0], self.unknown_cmd_a9[0]}:
            self.last_command_status = True

    def login(self):
        pin_data = bytearray(struct.pack(">H", self.pin))
        self.send_Zemismart_packet(self.pin_cmd, pin_data)

    def send_BLE_packet(self, handle, data):
        return handle.write(bytes(data), withResponse=False)

    def send_Zemismart_packet(self, command, data, wait_for_notification_time=2):
        self.last_command_status = None
        length = bytearray([len(data)])
        data_without_checksum = self.start_bytes + command + length + data
        data_with_checksum = data_without_checksum + self.calculate_checksum(data_without_checksum)
        if self.datahandle is None or self.device is None:
            print("datahandle or device is not defined. Did you use with statement?")
            return False
        else:
            write_result = self.send_BLE_packet(self.datahandle, data_with_checksum)
            if wait_for_notification_time > 0:
                start_time = time.time()
                while self.last_command_status is None and time.time() - wait_for_notification_time <= start_time:
                    if self.device.waitForNotifications(wait_for_notification_time) and self.last_command_status is not None:
                        return self.last_command_status is True
                return False
            return write_result

    def calculate_checksum(self, data):
        checksum = 0
        for byte in data:
            checksum = checksum ^ byte
        checksum = checksum ^ 0xff
        return bytearray([checksum])

    def open(self):
        return self.send_Zemismart_packet(self.move_cmd, self.open_data)

    def close(self):
        return self.send_Zemismart_packet(self.move_cmd, self.close_data)

    def stop(self):
        return self.send_Zemismart_packet(self.move_cmd, self.stop_data)

    def set_position(self, position):
        if 0 <= position <= 100:
            return self.send_Zemismart_packet(self.set_position_cmd, bytearray(struct.pack(">B", position)))

    def save_position(self, position):
        self.position = position

    def save_battery(self, battery):
        self.battery = battery

    def update(self):
        self._get_position_responses = set()
        if not self.send_Zemismart_packet(self.get_position_cmd, bytearray([0x01]), 1):
            return False
        elif not self.send_Zemismart_packet(self.get_battery_cmd, bytearray([0x01]), 1):
            return False
        else:
            return True

    def timer_toggle(self, timer_id, enabled):
        if timer_id >= len(self.timers):
            raise IndexError('Timer #%d not found' % timer_id)

        ret = self.send_Zemismart_packet(self.update_timer_cmd,
                                         bytearray([timer_id + 1, 0x00, int(enabled), 0x00, 0x00, 0x00, 0x00]),
                                         1)

        if ret:
            self.timers[timer_id - 1].enabled = enabled

        return ret

    def timer_delete(self, timer_id):
        if timer_id >= len(self.timers):
            raise IndexError('Timer #%d not found' % timer_id)

        ret = self.send_Zemismart_packet(self.update_timer_cmd,
                                         bytearray([timer_id + 1, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]),
                                         1)

        if ret:
            del self.timers[timer_id - 1]

        return ret

    def _update_timer_internal(self, timer_id, timer):
        return self.send_Zemismart_packet(self.update_timer_cmd,
                                          bytearray([timer_id + 1, 0x00, int(timer.enabled), int(timer.position),
                                                     int(timer.repeats), int(timer.hours), int(timer.minutes)]),
                                          1)

    def timer_update(self, timer_id, timer):
        if timer_id >= len(self.timers):
            raise IndexError('Timer #%d not found' % timer_id)

        ret = self._update_timer_internal(timer_id, timer)

        if ret:
            self.timers[timer_id] = timer

        return ret

    def timer_add(self, timer):
        if len(self.timers) >= 4:
            raise ValueError('You can have only 4 timers')

        ret = self._update_timer_internal(len(self.timers) - 1, timer)

        if ret:
            self.timers.append(timer)

        return ret
