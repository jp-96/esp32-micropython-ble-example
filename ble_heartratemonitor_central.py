import bluetooth
import struct
import time

from ble_advertising import decode_services, decode_name

from micropython import const

# _IRQ_CENTRAL_CONNECT = const(1)
# _IRQ_CENTRAL_DISCONNECT = const(2)
# _IRQ_GATTS_WRITE = const(3)
# _IRQ_GATTS_READ_REQUEST = const(4)
_IRQ_SCAN_RESULT = const(5)
_IRQ_SCAN_DONE = const(6)
_IRQ_PERIPHERAL_CONNECT = const(7)
_IRQ_PERIPHERAL_DISCONNECT = const(8)
_IRQ_GATTC_SERVICE_RESULT = const(9)
_IRQ_GATTC_SERVICE_DONE = const(10)
_IRQ_GATTC_CHARACTERISTIC_RESULT = const(11)
_IRQ_GATTC_CHARACTERISTIC_DONE = const(12)
_IRQ_GATTC_DESCRIPTOR_RESULT = const(13)
_IRQ_GATTC_DESCRIPTOR_DONE = const(14)
_IRQ_GATTC_READ_RESULT = const(15)
_IRQ_GATTC_READ_DONE = const(16)
_IRQ_GATTC_WRITE_DONE = const(17)
_IRQ_GATTC_NOTIFY = const(18)
# _IRQ_GATTC_INDICATE = const(19)
# _IRQ_GATTS_INDICATE_DONE = const(20)
# _IRQ_MTU_EXCHANGED = const(21)
# _IRQ_L2CAP_ACCEPT = const(22)
# _IRQ_L2CAP_CONNECT = const(23)
# _IRQ_L2CAP_DISCONNECT = const(24)
# _IRQ_L2CAP_RECV = const(25)
# _IRQ_L2CAP_SEND_READY = const(26)
_IRQ_CONNECTION_UPDATE = const(27)
# _IRQ_ENCRYPTION_UPDATE = const(28)
# _IRQ_GET_SECRET = const(29)
# _IRQ_SET_SECRET = const(30)

_ADV_IND = const(0x00)
_ADV_DIRECT_IND = const(0x01)
# _ADV_SCAN_IND = const(0x02)
# _ADV_NONCONN_IND = const(0x03)

_SERV_HRM_UUID = bluetooth.UUID(0x180D) # org.bluetooth.service.heart_rate.xml
_CHAR_HRM_UUID = bluetooth.UUID(0x2A37) # org.bluetooth.characteristic.heart_rate_measurement.xml
_DESC_CCC_UUID = bluetooth.UUID(0x2902) # org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
_CHAR_BSL_UUID = bluetooth.UUID(0x2A38) # org.bluetooth.characteristic.body_sensor_location.xml
_CHAR_HRC_UUID = bluetooth.UUID(0x2A39) # org.bluetooth.characteristic.heart_rate_control_point.xml

# decode_heart_rate_measurement t
_HRM_HRV = const(1) # Heart Rate Measurement Value
_HRM_SCS = const(2) # Sensor Contact Status
_HRM_EES = const(3) # Energy Expended
_HRM_RRI = const(4) # RR-Interval

def decode_heart_rate_measurement(b, t):
    # org.bluetooth.characteristic.heart_rate_measurement.xml
    
    # Flags Field
    flags = b[0]

    # bit:0   Heart Rate Value Format bit
    hrv_flag = flags & 1
    if t == _HRM_HRV:
        # Heart Rate Measurement Value - org.bluetooth.unit.period.beats_per_minute
        if hrv_flag == 0:
            # Heart Rate Measurement Value (uint8)
            hrv = b[1]
        else:
            # Heart Rate Measurement Value (uint16)
            hrv = b[1] | (b[2] << 8)
        return hrv

    # bit:2-1 Sensor Contact Status bits
    scs_flag = (flags >> 1) & 3
    if t == _HRM_SCS:
        return scs_flag
    
    # bit:3   Energy Expended Status bit
    ees_flag = (flags >> 3) & 1
    if t == _HRM_EES:
        # Energy Expended - org.bluetooth.unit.energy.joule
        eev = None
        if ees_flag == 1:
            idx = 2 + hrv_flag
            eev = b[idx] | (b[idx + 1] << 8)
        return eev

    # bit:4   RR-Interval bit
    rri_flag = (flags >> 4) & 1
    if t == _HRM_RRI:
        # RR-Interval - Resolution of 1/1024 second
        rr = []
        if rri_flag == 1:
            idx = 2 + hrv_flag + ees_flag * 2
            while idx < len(b):
                rr.append(b[idx] | (b[idx + 1] << 8))
                idx += 2
        return rr


def decode_heart_rate_value(b):
    # Heart Rate Measurement Value
    return decode_heart_rate_measurement(b, _HRM_HRV)

def decode_sensor_contact_status(b):
    # Sensor Contact Status
    return decode_heart_rate_measurement(b, _HRM_SCS)

def decode_sensor_contact_status_str(b):
    scs_flag = decode_sensor_contact_status(b)
    # Sensor Contact Status
    if scs_flag == 0:
        #scs = "0 - Sensor Contact feature is not supported in the current connection"
        scs = "0:Not supported"
    elif scs_flag == 1:
        #scs = "1 - Sensor Contact feature is not supported in the current connection"
        scs = "1:Not supported"
    elif scs_flag == 2:
        #scs = "2 - Sensor Contact feature is supported, but contact is not detected"
        scs = "2:Not detected"
    else:
        #scs = "3 - Sensor Contact feature is supported and contact is detected"
        scs = "3:Detected"
    return scs

def decode_energy_expended(b):
    # Energy Expended
    return decode_heart_rate_measurement(b, _HRM_EES)

def decode_rr_interval(b):
    # RR-Interval
    return decode_heart_rate_measurement(b, _HRM_RRI)

class BLEHeartRateMonitorCentral:
    def __init__(self, ble):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        self._reset()

    def _reset(self):
        # Cached name and address from a successful scan.
        self._name = None
        self._addr_type = None
        self._addr = None

        # Callbacks for completion of various operations.
        # These reset back to None after being invoked.
        self._scan_callback = None
        self._conn_callback = None
        self._read_callback = None
        self._write_callback = None

        # Persistent callback for when new data is notified from the device.
        self._notify_callback = None

        # Connected device.
        self._conn_handle = None
        self._start_handle = None
        self._end_handle = None

        # GATTC_CHARACTERISTIC
        self._notify_handle = None
        self._config_handle = None
        self._location_handle = None
        self._control_handle = None
        
        self._connected = False

    def _irq(self, event, data):
        print("_irq() event=", event)
        if event == _IRQ_SCAN_RESULT:
            # A single scan result, gap_scan().
            addr_type, addr, adv_type, rssi, adv_data = data
            if adv_type in (_ADV_IND, _ADV_DIRECT_IND) and _SERV_HRM_UUID in decode_services(adv_data):  # _SERV_HRM_UUID found.
                # Found a potential device, remember it and stop scanning.
                self._addr_type = addr_type
                self._addr = bytes(
                    addr
                )  # Note: addr buffer is owned by caller so need to copy it.
                self._name = decode_name(adv_data) or "?"
                self._ble.gap_scan(None)    # manually stop.
        elif event == _IRQ_SCAN_DONE:
            # Scan duration finished or manually stopped, gap_scan().
            if self._scan_callback:
                if self._addr:
                    # Found a device during the scan (and the scan was explicitly stopped).
                    self._scan_callback(self._addr_type, self._addr, self._name)
                    self._scan_callback = None
                else:
                    # Scan timed out.
                    self._scan_callback(None, None, None)
        elif event == _IRQ_PERIPHERAL_CONNECT:
            # A successful gap_connect().
            conn_handle, addr_type, addr = data
            if addr_type == self._addr_type and addr == self._addr:
                self._conn_handle = conn_handle
                self._ble.gattc_discover_services(self._conn_handle)
        elif event == _IRQ_PERIPHERAL_DISCONNECT:
            # Connected peripheral has disconnected.
            conn_handle, addr_type, addr = data
            if conn_handle == self._conn_handle:
                # If it was initiated by us, it'll already be reset.
                self._reset()
        elif event == _IRQ_GATTC_SERVICE_RESULT:
            # Called for each service found by gattc_discover_services().
            conn_handle, start_handle, end_handle, uuid = data
            if conn_handle == self._conn_handle and uuid == _SERV_HRM_UUID:  # _SERV_HRM_UUID found.
                self._start_handle, self._end_handle = start_handle, end_handle
        elif event == _IRQ_GATTC_SERVICE_DONE:
            # Called once service discovery is complete.
            # Note: Status will be zero on success, implementation-specific value otherwise.
            conn_handle, status = data
            if self._start_handle and self._end_handle:
                self._ble.gattc_discover_characteristics(
                    self._conn_handle, self._start_handle, self._end_handle
                )
            else:
                print("Failed to find service.")
        elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT:
            # Called for each characteristic found by gattc_discover_services().
            conn_handle, def_handle, value_handle, properties, uuid = data
            if conn_handle == self._conn_handle:
                if uuid == _CHAR_HRM_UUID: # _CHAR_HRM_UUID found.
                    self._notify_handle = value_handle
                if uuid == _CHAR_BSL_UUID: # _CHAR_BSL_UUID found.
                    self._location_handle = value_handle
                if uuid == _CHAR_HRC_UUID: # _CHAR_HRC_UUID found.
                    self._control_handle = value_handle
        elif event == _IRQ_GATTC_CHARACTERISTIC_DONE:
            # Called once service discovery is complete.
            # Note: Status will be zero on success, implementation-specific value otherwise.
            conn_handle, status = data
            if self._notify_handle :
                self._ble.gattc_discover_descriptors(self._conn_handle, self._notify_handle, self._end_handle)
            else:
                print("Failed to find characteristic.")
        elif event == _IRQ_GATTC_DESCRIPTOR_RESULT:
            # Called for each descriptor found by gattc_discover_descriptors().
            conn_handle, dsc_handle, uuid = data
            if conn_handle == self._conn_handle and uuid == _DESC_CCC_UUID:  # _DESC_CCC_UUID found.
                if not self._config_handle:
                    self._config_handle = dsc_handle
        elif event == _IRQ_GATTC_DESCRIPTOR_DONE:
            # Called once service discovery is complete.
            # Note: Status will be zero on success, implementation-specific value otherwise.
            conn_handle, status = data
            if self._config_handle:
                # We've finished connecting and discovering device, fire the connect callback.
                self._connected = True
                if self._conn_callback:
                    self._conn_callback()
            else:
                print("Failed to find descriptor.")
        elif event == _IRQ_GATTC_WRITE_DONE:
            # A gattc_write() has completed.
            # Note: The value_handle will be zero on btstack (but present on NimBLE).
            # Note: Status will be zero on success, implementation-specific value otherwise.
            conn_handle, value_handle, status = data
            if self._write_callback:
                self._write_callback(status)
                self._write_callback = None
        elif event == _IRQ_GATTC_READ_RESULT:
            # A read completed successfully.
            conn_handle, value_handle, char_data = data
            if conn_handle == self._conn_handle and value_handle == self._location_handle:
                if self._read_callback:
                    self._read_callback(bytes(char_data))
                    self._read_callback = None
        elif event == _IRQ_GATTC_READ_DONE:
            # Read completed (no-op).
            conn_handle, value_handle, status = data
        elif event == _IRQ_GATTC_NOTIFY:
            # A server has sent a notify request.
            conn_handle, value_handle, notify_data = data
            if conn_handle == self._conn_handle:
                if self._notify_callback:
                    self._notify_callback(value_handle, bytes(notify_data))
        elif event == _IRQ_CONNECTION_UPDATE:
            # The remote device has updated connection parameters.
            conn_handle, conn_interval, conn_latency, supervision_timeout, status = data
            print("_IRQ_CONNECTION_UPDATE")
        else:
            # Unhandled
            print("************ Unhandled ************ event=", event)

    # Returns true if we've successfully connected and discovered characteristics.
    def is_connected(self):
        return self._connected

    # Find a device advertising the environmental sensor service.
    def scan(self, callback=None):
        self._addr_type = None
        self._addr = None
        self._scan_callback = callback
        self._ble.gap_scan(2000, 30000, 30000)

    # Connect to the specified device (otherwise use cached address from a scan).
    def connect(self, addr_type=None, addr=None, callback=None):
        self._addr_type = addr_type or self._addr_type
        self._addr = addr or self._addr
        self._conn_callback = callback
        if self._addr_type is None or self._addr is None:
            return False
        self._ble.gap_connect(self._addr_type, self._addr)
        return True

    # Disconnect from current device.
    def disconnect(self):
        if not self._conn_handle:
            return
        self._ble.gap_disconnect(self._conn_handle)
        self._reset()
    
    def _gattc_read_sync(self, conn_handle, value_handle):
        result = None
        def on_read(v):
            nonlocal result
            result = v
        self._read_callback = on_read
        # Issues an (asynchronous) read, will invoke callback with data.
        self._ble.gattc_read(conn_handle, value_handle)
        counter = 0
        while self._read_callback and counter < 10:
            time.sleep_ms(50)
            counter += 1
        return result

    def _gattc_write_sync(self, conn_handle, value_handle, data, mode=0):
        result = None
        if mode == 1:
            def on_write(v):
                nonlocal result
                result = v
            self._write_callback = on_write
        self._ble.gattc_write(conn_handle, value_handle, data, mode)
        if mode == 1:
            counter = 0
            while self._write_callback and counter < 10:
                time.sleep_ms(50)
                counter += 1
            return result

    # Enable device notifications, and sets a callback to be invoked when the device notifies us.
    def enable_notify(self, callback):
        self._notify_callback = callback
        if self.is_connected():
            result = self._gattc_write_sync(self._conn_handle, self._config_handle, struct.pack('<h', 1), 1)
            print("enable_notify:", result)
    
    # Body sensor location
    def read_body_sensor_location(self):
        if not self.is_connected():
            return
        if self._location_handle:
            return self._gattc_read_sync(self._conn_handle, self._location_handle)

    # Energy Expended (not tested)
    def enable_energy(self):
        if not self.is_connected():
            return
        if self._control_handle:
            result = self._gattc_write_sync(self._conn_handle, self._control_handle, struct.pack('b', 1), 1)
            print("enable_energy:", result)

def demo():
    
    ble = bluetooth.BLE()
    central = BLEHeartRateMonitorCentral(ble)

    def execute():
        
        not_found = False

        def on_scan(addr_type, addr, name):
                
            def b2s(b):
                if b:
                    s=[]
                    for v in b:
                        s.append("{:02x}".format(v).upper())
                    return ":".join(s)
                else:
                    return ""

            if addr_type is not None:
                print("Found device:", addr_type, b2s(addr), "'" + name + "'")
                central.connect()
            else:
                nonlocal not_found
                not_found = True
                print("No device found.")

        central.scan(callback=on_scan)

        # Wait for connection...
        retry_count = 50
        while not central.is_connected():
            time.sleep_ms(100)
            if not_found:
                return
            retry_count = retry_count - 1
            if retry_count < 0:
                print("No connection.")
                return

        print("Connected")

        def on_notify(value_handle, notify_data):
            # print("on_notify()", value_handle, notify_data)
            
            print("  1)", decode_heart_rate_value(notify_data), "bpm")
            print("  2)", decode_sensor_contact_status_str(notify_data))
            print("  3)", decode_energy_expended(notify_data), "joule")
            print("  4)", decode_rr_interval(notify_data), "RR (1/1024 sec)")

            # org.bluetooth.characteristic.heart_rate_measurement.xml
            idx = 0

            # Flags Field
            flags = notify_data[idx]
            idx += 1
            # bit:0   Heart Rate Value Format bit
            hrv_flag = (flags    ) & 1
            # bit:2-1 Sensor Contact Status bits
            scs_flag = (flags >> 1) & 3
            # bit:3   Energy Expended Status bit
            ees_flag = (flags >> 3) & 1
            # bit:4   RR-Interval bit
            rri_flag = (flags >> 4) & 1

            # Heart Rate Measurement Value - org.bluetooth.unit.period.beats_per_minute
            if hrv_flag == 0:
                # Heart Rate Measurement Value (uint8)
                hrv = notify_data[idx]
                idx += 1
            else:
                # Heart Rate Measurement Value (uint16)
                hrv = notify_data[idx] | (notify_data[idx + 1] << 8)
                idx += 2
            
            # Sensor Contact Status
            scs = None
            if scs_flag == 0:
                #scs = "0 - Sensor Contact feature is not supported in the current connection"
                scs = "0:Not supported"
            elif scs_flag == 1:
                #scs = "1 - Sensor Contact feature is not supported in the current connection"
                scs = "1:Not supported"
            elif scs_flag == 2:
                #scs = "2 - Sensor Contact feature is supported, but contact is not detected"
                scs = "2:Not detected"
            else:
                #scs = "3 - Sensor Contact feature is supported and contact is detected"
                scs = "3:Detected"
            
            # Energy Expended - org.bluetooth.unit.energy.joule
            eev = None
            if ees_flag == 1:
                eev = notify_data[idx] | (notify_data[idx + 1] << 8)
                idx += 2 

            # RR-Interval - Resolution of 1/1024 second
            rr = []
            if rri_flag == 1:
                while idx < len(notify_data):
                    rr.append(notify_data[idx] | (notify_data[idx + 1] << 8))
                    idx += 2
            
            print("Heart Rate Monitor:", hrv, "bpm ,", scs, ",", eev, ",", rr)

        # enable notify
        central.enable_notify(callback=on_notify)

        # Body sensor location
        loc = central.read_body_sensor_location()
        loc_name = ["0-Other", "1-Chest", "2-Wrist", "3-Finger", "4-Hand", "5-Ear Lobe", "6-Foot"]
        print("-----------------------------------------")
        print(" body sensor location:", loc_name[loc[0]])
        print("-----------------------------------------")
        
        # Enable energy expanded
        central.enable_energy()

        # connection loop
        while central.is_connected():
            time.sleep_ms(100)
        
        print("Disconnected")

    try:
        while True:
            execute()
    except:
        ble.active(False)

if __name__ == "__main__":
    demo()
