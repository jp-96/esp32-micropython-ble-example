import bluetooth
import random
import struct
import time
import micropython

from ble_advertising import decode_services, decode_name

from micropython import const

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)
_IRQ_GATTS_READ_REQUEST = const(4)
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
_IRQ_GATTC_INDICATE = const(19)
_IRQ_GATTS_INDICATE_DONE = const(20)
_IRQ_MTU_EXCHANGED = const(21)
_IRQ_L2CAP_ACCEPT = const(22)
_IRQ_L2CAP_CONNECT = const(23)
_IRQ_L2CAP_DISCONNECT = const(24)
_IRQ_L2CAP_RECV = const(25)
_IRQ_L2CAP_SEND_READY = const(26)
_IRQ_CONNECTION_UPDATE = const(27)
_IRQ_ENCRYPTION_UPDATE = const(28)
_IRQ_GET_SECRET = const(29)
_IRQ_SET_SECRET = const(30)

_ADV_IND = const(0x00)
_ADV_DIRECT_IND = const(0x01)
_ADV_SCAN_IND = const(0x02)
_ADV_NONCONN_IND = const(0x03)

_SERVICE_UUID = bluetooth.UUID(0x1812)          # HID device/service
_CHARACTERISTIC_UUID = bluetooth.UUID(0x2a4d)   # REPORT characteristic(s)
_DESCRIPTOR_UUID = bluetooth.UUID(0x2902)       # REPORT descriptor(s)

class BLEABShutter3HidCentral:
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

        # Persistent callback for when new data is notified from the device.
        self._notify_callback = None

        # Connected device.
        self._conn_handle = None
        self._start_handle = None
        self._end_handle = None

        # GATTC_CHARACTERISTIC
        self._report1_handle = None
        self._report1_dsc_handle = None
        self._report2_handle = None
        self._report2_dsc_handle = None

        self._connected = False

    def _irq(self, event, data):
        print("_irq() event=", event)
        if event == _IRQ_CENTRAL_CONNECT:
            # A central has connected to this peripheral.
            conn_handle, addr_type, addr = data
        elif event == _IRQ_CENTRAL_DISCONNECT:
            # A central has disconnected from this peripheral.
            conn_handle, addr_type, addr = data
        elif event == _IRQ_GATTS_WRITE:
            # A client has written to this characteristic or descriptor.
            conn_handle, attr_handle = data
        elif event == _IRQ_GATTS_READ_REQUEST:
            # A client has issued a read. Note: this is only supported on STM32.
            # Return a non-zero integer to deny the read (see below), or zero (or None)
            # to accept the read.
            conn_handle, attr_handle = data
        elif event == _IRQ_SCAN_RESULT:
            # A single scan result, gap_scan().
            addr_type, addr, adv_type, rssi, adv_data = data
            if adv_type in (_ADV_IND, _ADV_DIRECT_IND) and _SERVICE_UUID in decode_services(adv_data):  # _SERVICE_UUID found.
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
            if conn_handle == self._conn_handle and uuid == _SERVICE_UUID:  # _SERVICE_UUID found.
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
                if uuid == _CHARACTERISTIC_UUID: # _CHARACTERISTIC_UUID found.
                    if not self._report1_handle:
                        self._report1_handle = value_handle
                    elif not self._report2_handle:
                        self._report2_handle = value_handle
                    else:
                        print("Too many characteristics.")
        elif event == _IRQ_GATTC_CHARACTERISTIC_DONE:
            # Called once service discovery is complete.
            # Note: Status will be zero on success, implementation-specific value otherwise.
            conn_handle, status = data
            if self._report1_handle and self._report2_handle:
                self._ble.gattc_discover_descriptors(self._conn_handle, self._start_handle, self._end_handle)
            else:
                print("Failed to find characteristic.")
        elif event == _IRQ_GATTC_DESCRIPTOR_RESULT:
            # Called for each descriptor found by gattc_discover_descriptors().
            conn_handle, dsc_handle, uuid = data
            if conn_handle == self._conn_handle and uuid == _DESCRIPTOR_UUID:  # _DESCRIPTOR_UUID found.
                if not self._report1_dsc_handle and self._report1_handle < dsc_handle and self._report2_handle > dsc_handle:
                    self._report1_dsc_handle = dsc_handle
                elif not self._report2_dsc_handle and self._report2_handle < dsc_handle:
                    self._report2_dsc_handle = dsc_handle
        elif event == _IRQ_GATTC_DESCRIPTOR_DONE:
            # Called once service discovery is complete.
            # Note: Status will be zero on success, implementation-specific value otherwise.
            conn_handle, status = data
            # We've finished connecting and discovering device, fire the connect callback.
            self._connected = True
            if self._conn_callback:
                self._conn_callback()
        elif event == _IRQ_GATTC_READ_RESULT:
            # A gattc_read() has completed.
            conn_handle, value_handle, char_data = data
        elif event == _IRQ_GATTC_READ_DONE:
            # A gattc_read() has completed.
            # Note: The value_handle will be zero on btstack (but present on NimBLE).
            # Note: Status will be zero on success, implementation-specific value otherwise.
            conn_handle, value_handle, status = data
        elif event == _IRQ_GATTC_WRITE_DONE:
            # A gattc_write() has completed.
            # Note: The value_handle will be zero on btstack (but present on NimBLE).
            # Note: Status will be zero on success, implementation-specific value otherwise.
            conn_handle, value_handle, status = data
        elif event == _IRQ_GATTC_NOTIFY:
            # A server has sent a notify request.
            conn_handle, value_handle, notify_data = data
            if conn_handle == self._conn_handle:
                if self._notify_callback:
                    self._notify_callback(value_handle, bytes(notify_data))
        elif event == _IRQ_GATTC_INDICATE:
            # A server has sent an indicate request.
            conn_handle, value_handle, notify_data = data
        elif event == _IRQ_GATTS_INDICATE_DONE:
            # A client has acknowledged the indication.
            # Note: Status will be zero on successful acknowledgment, implementation-specific value otherwise.
            conn_handle, value_handle, status = data
        elif event == _IRQ_MTU_EXCHANGED:
            # ATT MTU exchange complete (either initiated by us or the remote device).
            conn_handle, mtu = data
        elif event == _IRQ_L2CAP_ACCEPT:
            # A new channel has been accepted.
            # Return a non-zero integer to reject the connection, or zero (or None) to accept.
            conn_handle, cid, psm, our_mtu, peer_mtu = data
        elif event == _IRQ_L2CAP_CONNECT:
            # A new channel is now connected (either as a result of connecting or accepting).
            conn_handle, cid, psm, our_mtu, peer_mtu = data
        elif event == _IRQ_L2CAP_DISCONNECT:
            # Existing channel has disconnected (status is zero), or a connection attempt failed (non-zero status).
            conn_handle, cid, psm, status = data
        elif event == _IRQ_L2CAP_RECV:
            # New data is available on the channel. Use l2cap_recvinto to read.
            conn_handle, cid = data
        elif event == _IRQ_L2CAP_SEND_READY:
            # A previous l2cap_send that returned False has now completed and the channel is ready to send again.
            # If status is non-zero, then the transmit buffer overflowed and the application should re-send the data.
            conn_handle, cid, status = data
        elif event == _IRQ_CONNECTION_UPDATE:
            # The remote device has updated connection parameters.
            conn_handle, conn_interval, conn_latency, supervision_timeout, status = data
        elif event == _IRQ_ENCRYPTION_UPDATE:
            # The encryption state has changed (likely as a result of pairing or bonding).
            conn_handle, encrypted, authenticated, bonded, key_size = data
        elif event == _IRQ_GET_SECRET:
            # Return a stored secret.
            # If key is None, return the index'th value of this sec_type.
            # Otherwise return the corresponding value for this sec_type and key.
            sec_type, index, key = data
            return value
        elif event == _IRQ_SET_SECRET:
            # Save a secret to the store for this sec_type and key.
            sec_type, key, value = data
            return True
        elif event == _IRQ_PASSKEY_ACTION:
            # Respond to a passkey request during pairing.
            # See gap_passkey() for details.
            # action will be an action that is compatible with the configured "io" config.
            # passkey will be non-zero if action is "numeric comparison".
            conn_handle, action, passkey = data

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
    
    # Enable device notifications, and sets a callback to be invoked when the device notifies us.
    def enable_notify(self, callback):
        self._notify_callback = callback
        if self.is_connected():
            self._ble.gattc_write(self._conn_handle, self._report1_dsc_handle, struct.pack('<h', 1), 1)
            self._ble.gattc_write(self._conn_handle, self._report2_dsc_handle, struct.pack('<h', 1), 1)
            

def demo():
    
    ble = bluetooth.BLE()
    central = BLEABShutter3HidCentral(ble)

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
            print("on_notify()", value_handle, notify_data, "DOWN" if notify_data[0]==2 else "UP" )

        # enable notify
        central.enable_notify(callback=on_notify)

        # connection loop
        while central.is_connected():
            time.sleep_ms(100)
        
        print("Disconnected")

    while True:
        execute()

if __name__ == "__main__":
    demo()
