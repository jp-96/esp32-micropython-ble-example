import bluetooth
import random
import struct
import time
from ble_advertising import advertising_payload, decode_services, decode_name

from micropython import const

_IRQ_CONNECTION_UPDATE = const(27)

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTC_INDICATE = const(19)
_IRQ_GATTS_INDICATE_DONE = const(20)

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
_IRQ_GATTC_WRITE_DONE = const(17)
_IRQ_GATTC_NOTIFY = const(18)

_ADV_IND = const(0x00)
_ADV_DIRECT_IND = const(0x01)

# org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
_DESC_CCC_UUID = bluetooth.UUID(0x2902)

# org.bluetooth.characteristic.heart_rate_measurement.xml
_CHAR_HRM_UUID = bluetooth.UUID(0x2A37)
_CHAR_HRM = (
    _CHAR_HRM_UUID,
    bluetooth.FLAG_NOTIFY,
)

# org.bluetooth.service.heart_rate.xml
_SERVICE_HEART_RATE_UUID = bluetooth.UUID(0x180D)
_SERVICE_HEART_RATE = (
    _SERVICE_HEART_RATE_UUID,
    (
        _CHAR_HRM,
    ),
)

# org.bluetooth.characteristic.gap.appearance.xml
_ADV_APPEARANCE_GENERIC_HEARTRATE_SENSOR = const(832)

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

def b2s(b):
    if b:
        s=[]
        for v in b:
            s.append("{:02x}".format(v).upper())
        return ":".join(s)
    else:
        return ""
        
class BLEHrmSplitter:
    def __init__(self, ble, name="HRM_SPLIT"):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        
        # server(peripheral)
        ((self._handle,),) = self._ble.gatts_register_services((_SERVICE_HEART_RATE,))
        self._connections = set()
        self._payload = advertising_payload(
            name=name, services=[_SERVICE_HEART_RATE_UUID], appearance=_ADV_APPEARANCE_GENERIC_HEARTRATE_SENSOR
        )

        # client(central)
        self._addr_type = None
        self._addr = None
        
        # Callbacks for completion of various operations.
        # These reset back to None `before`` being invoked.
        self._scan_callback = None
        self._conn_callback = None
        self._disconn_callback = None
        self._discover_callback = None
        
        # Connected device.
        self._conn_handle = None    # connected
        self._start_handle = None
        self._end_handle = None
        self._value_handle = None
        self._config_handle = None  # connected

    def _irq(self, event, data):
        if event == _IRQ_CONNECTION_UPDATE:
            # The remote device has updated connection parameters.
            conn_handle, conn_interval, conn_latency, supervision_timeout, status = data
        
        elif event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
        
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            self._connections.remove(conn_handle)
            # Start advertising again to allow a new connection.
            self.advertise()

        elif event == _IRQ_SCAN_RESULT:
            addr_type, addr, adv_type, rssi, adv_data = data
            if adv_type in (_ADV_IND, _ADV_DIRECT_IND) and _SERVICE_HEART_RATE_UUID in decode_services(
                adv_data
            ):
                # Found a potential device, remember it and stop scanning.
                self._addr_type = addr_type
                self._addr = bytes(
                    addr
                )  # Note: addr buffer is owned by caller so need to copy it.
                self._name = decode_name(adv_data) or "?"
                self._ble.gap_scan(None)

        elif event == _IRQ_SCAN_DONE:
            if self._scan_callback:
                callback = self._scan_callback
                self._scan_callback = None
                if self._addr:
                    # Found a device during the scan (and the scan was explicitly stopped).
                    callback(self._addr_type, self._addr, self._name)
                else:
                    # Scan timed out.
                    callback(None, None, None)
        
        elif event == _IRQ_PERIPHERAL_CONNECT:
            # Connect successful.
            conn_handle, addr_type, addr = data
            if addr_type == self._addr_type and addr == self._addr:
                self._conn_handle = conn_handle
                if self._conn_callback:
                    callback = self._conn_callback
                    self._conn_callback = None
                    callback()
                self._ble.gattc_discover_services(self._conn_handle)

        elif event == _IRQ_PERIPHERAL_DISCONNECT:
            # Disconnect (either initiated by us or the remote end).
            conn_handle, _, _ = data
            if conn_handle == self._conn_handle:
                callback = self._disconn_callback
                self._conn_callback = None
                self._disconn_callback = None
                self._discover_callback = None
                self._conn_handle = None
                self._start_handle = None
                self._end_handle = None
                self._value_handle = None
                self._config_handle = None
                if callback:
                    callback()

        elif event == _IRQ_GATTC_SERVICE_RESULT:
            # Connected device returned a service.
            conn_handle, start_handle, end_handle, uuid = data
            if conn_handle == self._conn_handle and uuid == _SERVICE_HEART_RATE_UUID:
                self._start_handle, self._end_handle = start_handle, end_handle

        elif event == _IRQ_GATTC_SERVICE_DONE:
            # Service query complete.
            if self._start_handle and self._end_handle:
                self._ble.gattc_discover_characteristics(
                    self._conn_handle, self._start_handle, self._end_handle
                )
            else:
                print("Failed to find the service.")
                # We've finished connecting and discovering device, fire the connect callback.
                if self._discover_callback:
                    callback = self._discover_callback
                    self._discover_callback = None
                    callback(False)

        elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT:
            # Connected device returned a characteristic.
            conn_handle, def_handle, value_handle, properties, uuid = data
            if conn_handle == self._conn_handle and uuid == _CHAR_HRM_UUID:
                self._value_handle = value_handle

        elif event == _IRQ_GATTC_CHARACTERISTIC_DONE:
            # Characteristic query complete.
            if self._value_handle:
                self._ble.gattc_discover_descriptors(
                    self._conn_handle, self._start_handle, self._end_handle
                )
            else:
                print("Failed to find characteristic.")
                # We've finished connecting and discovering device, fire the connect callback.
                if self._discover_callback:
                    callback = self._discover_callback
                    self._discover_callback = None
                    callback(False)

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
                result = True
            else:
                print("Failed to find descriptor.")
                result = False
            # We've finished connecting and discovering device, fire the connect callback.
            if self._discover_callback:
                callback = self._discover_callback
                self._discover_callback = None
                callback(result)

        elif event == _IRQ_GATTC_WRITE_DONE:
            # A gattc_write() has completed.
            # Note: The value_handle will be zero on btstack (but present on NimBLE).
            # Note: Status will be zero on success, implementation-specific value otherwise.
            conn_handle, value_handle, status = data
        
        elif event == _IRQ_GATTC_NOTIFY:
            # A server has sent a notify request.
            conn_handle, value_handle, notify_data = data
            for conn_handle in self._connections:
                self._ble.gatts_notify(conn_handle, self._handle, notify_data)
            if decode_sensor_contact_status(notify_data) == 3:
                bpm = decode_heart_rate_value(notify_data)
            else:
                bpm = 0
            print("HeartRate(bpm):", bpm, ", splitter:", len(self._connections))
        else:
            print("_irq", event)
        
    def advertise(self, interval_us=500000):
        print("Advertising...")
        self._ble.gap_advertise(interval_us, adv_data=self._payload)

    # Find a device advertising the environmental sensor service.
    def scan(self, scan_callback=None):
        print("Scanning...")
        self._addr_type = None
        self._addr = None
        self._scan_callback = scan_callback
        self._ble.gap_scan(2000, 30000, 30000)
    
    # Returns true if we've successfully connected and discovered characteristics.
    def is_connected(self):
        return self._conn_handle is not None and self._config_handle is not None
    
    # Connect to the specified device (otherwise use cached address from a scan).
    def connect(self, addr_type=None, addr=None, conn_callback=None, disconn_callback=None, discover_callback=None):
        self._addr_type = addr_type or self._addr_type
        self._addr = addr or self._addr
        if self._addr_type is None or self._addr is None:
            print("Rrror - connect")
            return False
        print("Connecting...")
        self._conn_callback = conn_callback
        self._disconn_callback = disconn_callback
        self._discover_callback = discover_callback
        self._conn_handle = None
        self._start_handle = None
        self._end_handle = None
        self._value_handle = None
        self._ble.gap_connect(self._addr_type, self._addr)
        return True
    
    # Disconnect from current device.
    def disconnect(self):
        if not self._conn_handle:
            return
        self._ble.gap_disconnect(self._conn_handle)
    
    # Enable device notifications
    def enable_notify(self):
        if self.is_connected():
            self._ble.gattc_write(self._conn_handle, self._config_handle, struct.pack('<h', 1), 1)

def demo():
     
    ble = bluetooth.BLE()
    hrm = BLEHrmSplitter(ble)
    hrm.advertise()

    def on_discover(done):
        if done:
            hrm.enable_notify()
        else:
            hrm.disconnect()
    
    def on_disconnect():
        print("Disconnect")
        hrm.scan(scan_callback=on_scan)
    
    def on_connect():
        print("Connected")

    def on_scan(addr_type, addr, name):
        if addr:
            print("scan:", "'" + name + "'", "addr_type=", addr_type, "addr=", addr, b2s(addr))
            if hrm.connect(conn_callback=on_connect, disconn_callback=on_disconnect, discover_callback=on_discover):
                return
        hrm.scan(scan_callback=on_scan)

    hrm.scan(scan_callback=on_scan)

    while True:
        time.sleep_ms(1000)


if __name__ == "__main__":
    demo()
