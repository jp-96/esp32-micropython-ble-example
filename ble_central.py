import bluetooth
import struct
import time
import micropython

from ble_advertising import decode_services, decode_name

from micropython import const

def b2s(b):
    if b:
        s=[]
        for v in b:
            s.append("{:02x}".format(v).upper())
        return ":".join(s)
    else:
        return ""

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

class BLEScanner:
    def __init__(self, ble):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        self._reset()

    def _reset(self):
        # Cached name and address from a successful scan.
        self._device_dict = {}  # key: bytes([adv_type]) + bytes(addr), value: [adv_type, addr, name]
        
        # scanning filter
        self._scan_filter_adv_types = None 
        self._scan_filter_address_str = None
        self._scan_filter_uuid = None
        self._scan_list_all = False # scan devices all if True.

        # Cached handles of conected devices.
        # <<gatt_type>> = 0-service, 1-char, 2-desc
        # <<gattc>> = {"type":<<gatt_type>>, "data":<<data>>}
        # <<gattcs>> = {key: <def_handle>||<desc_handle>, value: <<gatt>>} 
        # <<service>> = {"type":<<gatt_type>>, "data":<<data>>, "gattcs":<<gattcs>>}
        self._conn_dict = {} # {key: <<connectinon handle>>, value:{"handle":<<connectinon handle>>, "services":[<<service>>,...], "handles":{}}}

        # connecting var.
        self._conn_addr_type = None 
        self._conn_address = None
        self._conn_handle = None        # Connecting handle
        self._conn_connection = None
        self._conn_services_count = 0
        self._conn_service_idx = 0
        self._conn_service = None

        # Callbacks for completion of various operations.
        # These reset back to None after being invoked.
        self._scan_callback = None
        self._conn_callback = None
        self._read_callback = None
        self._write_callback = None

        self._is_busy = False

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
            addr_str = b2s(bytes(addr))
            if (self._scan_filter_adv_types == None or adv_type in self._scan_filter_adv_types) \
            and (self._scan_filter_address_str == None or addr_str == self._scan_filter_address_str) \
            and (self._scan_filter_uuid == None or self._scan_filter_uuid in decode_services(adv_data)):
                # Found a potential device, remember it and stop scanning.
                addr_bytes = bytes(
                    addr
                )  # Note: addr buffer is owned by caller so need to copy it.
                key = bytes([addr_type]) + addr_bytes
                name = decode_name(adv_data) or "?"
                self._device_dict[key] = (addr_type, addr_bytes, adv_type, name)
                if not self._scan_list_all:
                    self._ble.gap_scan(None)    # manually stop.
        elif event == _IRQ_SCAN_DONE:
            # Scan duration finished or manually stopped, gap_scan().
            self._complete_scan()
        elif event == _IRQ_PERIPHERAL_CONNECT:
            # A successful gap_connect().
            conn_handle, addr_type, addr = data
            if addr_type == self._conn_addr_type and addr == self._conn_address:
                self._conn_handle = conn_handle
                self._conn_connection = {"handle":conn_handle, "services":[], "handles":{}} 
                self._conn_dict[conn_handle] = self._conn_connection
                self._ble.gattc_discover_services(conn_handle)
        elif event == _IRQ_PERIPHERAL_DISCONNECT:
            # Connected peripheral has disconnected.
            conn_handle, addr_type, addr = data
            if self._conn_handle == conn_handle:
                self._conn_handle = None
                self._conn_connection = None
            self._conn_dict[conn_handle] = None
            del self._conn_dict[conn_handle]
        elif event == _IRQ_GATTC_SERVICE_RESULT:
            # Called for each service found by gattc_discover_services().
            conn_handle, start_handle, end_handle, uuid = data
            if conn_handle == self._conn_handle:
                service = {"type":0, "handle":start_handle, "data":(conn_handle, start_handle, end_handle, bluetooth.UUID(uuid)), "gattcs":{}}
                self._conn_connection["services"].append(service)
        elif event == _IRQ_GATTC_SERVICE_DONE:
            # Called once service discovery is complete.
            # Note: Status will be zero on success, implementation-specific value otherwise.
            conn_handle, status = data
            if conn_handle == self._conn_handle:
                self._conn_services_count = len(self._conn_connection["services"])
                self._conn_service_idx = 0
                if self._conn_services_count > self._conn_service_idx:
                    self._conn_service = self._conn_connection["services"][self._conn_service_idx]
                    conn_handle, start_handle, end_handle, uuid = self._conn_service["data"]
                    self._ble.gattc_discover_characteristics(conn_handle, start_handle, end_handle)
                else:
                    print("Failed to find service.")
                    # We've finished connecting and discovering device, fire the connect callback.
                    self._complete_connect()
        elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT:
            # Called for each characteristic found by gattc_discover_services().
            conn_handle, def_handle, value_handle, properties, uuid = data
            if conn_handle == self._conn_handle:
                gattc = {"type":1.1, "handle":def_handle, "data":(conn_handle, def_handle, value_handle, properties, bluetooth.UUID(uuid))}
                self._conn_service["gattcs"][def_handle] = gattc
                self._conn_connection["handles"][def_handle] = {}
                gattc = {"type":1.2, "handle":value_handle, "data":(conn_handle, def_handle, value_handle, properties, bluetooth.UUID(uuid))}
                self._conn_service["gattcs"][value_handle] = gattc
                self._conn_connection["handles"][value_handle] = {}
        elif event == _IRQ_GATTC_CHARACTERISTIC_DONE:
            # Called once service discovery is complete.
            # Note: Status will be zero on success, implementation-specific value otherwise.
            conn_handle, status = data
            if conn_handle == self._conn_handle:
                self._conn_service_idx += 1
                if self._conn_services_count > self._conn_service_idx:
                    self._conn_service = self._conn_connection["services"][self._conn_service_idx]
                    conn_handle, start_handle, end_handle, uuid = self._conn_service["data"]
                    self._ble.gattc_discover_characteristics(conn_handle, start_handle, end_handle)
                else:
                    self._conn_services_count = len(self._conn_connection["services"])
                    self._conn_service_idx = 0
                    if self._conn_services_count > self._conn_service_idx:
                        self._conn_service = self._conn_connection["services"][self._conn_service_idx]
                        conn_handle, start_handle, end_handle, uuid = self._conn_service["data"]
                        self._ble.gattc_discover_descriptors(conn_handle, start_handle, end_handle)
        elif event == _IRQ_GATTC_DESCRIPTOR_RESULT:
            # Called for each descriptor found by gattc_discover_descriptors().
            conn_handle, dsc_handle, uuid = data
            if conn_handle == self._conn_handle:
                gattc = {"type":2, "handle":dsc_handle, "data":(conn_handle, dsc_handle, bluetooth.UUID(uuid))}
                if not dsc_handle in self._conn_service["gattcs"].keys():
                    self._conn_service["gattcs"][dsc_handle] = gattc
                    self._conn_connection["handles"][dsc_handle] = {}
        elif event == _IRQ_GATTC_DESCRIPTOR_DONE:
            # Called once service discovery is complete.
            # Note: Status will be zero on success, implementation-specific value otherwise.
            conn_handle, status = data
            if conn_handle == self._conn_handle:
                self._conn_service_idx += 1
                if self._conn_services_count > self._conn_service_idx:
                    self._conn_service = self._conn_connection["services"][self._conn_service_idx]
                    conn_handle, start_handle, end_handle, uuid = self._conn_service["data"]
                    self._ble.gattc_discover_descriptors(conn_handle, start_handle, end_handle)
                else:
                    # We've finished connecting and discovering device, fire the connect callback.
                    self._complete_connect()
        elif event == _IRQ_GATTC_READ_RESULT:
            # A read completed successfully.
            conn_handle, value_handle, char_data = data
            if conn_handle == self._conn_handle:
                if self._read_callback:
                    self._read_callback(bytes(char_data))
                    self._read_callback = None
                self._conn_handle = None
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
            if conn_handle == self._conn_handle:
                if self._write_callback:
                    self._write_callback(status)
                    self._write_callback = None
                self._conn_handle = None
        elif event == _IRQ_GATTC_NOTIFY:
            # A server has sent a notify request.
            conn_handle, value_handle, notify_data = data
            callback = None
            try:
                callback = self._conn_dict[conn_handle]["handles"][value_handle]["notify_callback"]
            except:
                pass
            if callback:
                callback(conn_handle, value_handle, bytes(notify_data))
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
    
    # Returns true if ble busy.
    def is_busy(self):
        return self._is_busy

    # Find a device advertising the environmental sensor service.
    def scan(self, adv_types = (_ADV_IND, _ADV_DIRECT_IND), address = None, service_uuid = None, list_all = False, callback=None):
        self._is_busy = True
        self._device_dict = {}
        self._scan_filter_adv_types = adv_types
        self._scan_filter_address_str = address
        self._scan_filter_uuid = service_uuid
        self._scan_list_all = list_all
        self._scan_callback = callback
        self._ble.gap_scan(2000, 30000, 30000)
    
    def _complete_scan(self):
        devices = list(self._device_dict.values())
        if self._scan_callback:
            # Found devices during the scan (and the scan was explicitly stopped).
            # Or scan timed out.
            self._scan_callback(devices)
            self._scan_callback = None
        self._scan_filter_adv_types = None 
        self._scan_filter_address_str = None
        self._scan_filter_uuid = None
        self._scan_list_all = False # scan devices all if True.
        self._is_busy = False
    
    # Connect to the specified device (otherwise use cached address from a scan).
    def connect(self, addr_type, addr, callback=None):
        self._is_busy = True
        self._conn_addr_type = addr_type
        self._conn_address = addr
        self._conn_handle = None
        self._conn_connection = None
        self._conn_callback = callback
        self._ble.gap_connect(addr_type, addr)

    def _complete_connect(self):
        connection = self._conn_connection
        if self._conn_callback:
            self._conn_callback(connection)
        self._conn_addr_type = None 
        self._conn_address = None
        self._conn_handle = None        # Connecting handle
        self._conn_connection = None
        self._conn_services_count = 0
        self._conn_service_idx = 0
        self._conn_service = None
        self._is_busy = False
    
    def is_connected(self, conn_handle):
        return conn_handle in self._conn_dict.keys()

    # Disconnect from current device.
    def disconnect(self, conn_handle):
        self._ble.gap_disconnect(conn_handle)
    
    # Sets a callback to be invoked when the device notifies us.
    def register_notify(self, conn_handle, value_handle, callback):
        self._conn_dict[conn_handle]["handles"][value_handle]["notify_callback"] = callback

    def gattc_read_sync(self, conn_handle, value_handle):
        result = None
        def on_read(v):
            nonlocal result
            result = v[0]
        self._read_callback = on_read
        self._conn_handle = conn_handle
        # Issues an (asynchronous) read, will invoke callback with data.
        self._ble.gattc_read(conn_handle, value_handle)
        counter = 0
        while self._read_callback and counter < 10:
            time.sleep_ms(50)
            counter += 1
        return result
    
    def gattc_write_sync(self, conn_handle, value_handle, data, mode=0):
        result = None
        if mode == 1:
            def on_write(v):
                nonlocal result
                result = v
            self._write_callback = on_write
            self._conn_handle = conn_handle
        self._ble.gattc_write(conn_handle, value_handle, data, mode)
        if mode == 1:
            counter = 0
            while self._write_callback and counter < 10:
                time.sleep_ms(50)
                counter += 1
            return result

def demo():
    
    _SERVICE_UUID = bluetooth.UUID(0x1812)          # HID device/service
    _CHARACTERISTIC_UUID = bluetooth.UUID(0x2a4d)   # REPORT characteristic(s)
    _DESCRIPTOR_UUID = bluetooth.UUID(0x2902)       # REPORT descriptor(s)

    ble = bluetooth.BLE()
    central = BLEScanner(ble)

    def execute():
        
        print("Scanning...")
        not_found = False
        addr_type = None
        addr_bytes = None
        def on_scan(devices):
            nonlocal addr_type
            nonlocal addr_bytes
            if len(devices) == 0:
                nonlocal not_found
                not_found = True
                print("No device found")
                return
            
            for device in devices:
                addr_type, addr_bytes, adv_type, name = device
                print("Found device:", addr_type, addr_bytes, adv_type, "'" + name + "'")
            
            addr_type, addr_bytes, adv_type, name = devices[0]

        central.scan(service_uuid=_SERVICE_UUID, callback=on_scan)

        # Wait for connection...
        retry_count = 50
        while central.is_busy():
            time.sleep_ms(100)
            if not_found:
                return
            retry_count = retry_count - 1
            if retry_count < 0:
                print("Timeout.", retry_count)
                return
        
        print("Connecting...")
        conn_handle = None
        notify_handle1 = None
        desc_handle1 = None
        notify_handle2 = None
        desc_handle2 = None
        def on_conn(v):
            nonlocal conn_handle
            nonlocal notify_handle1
            nonlocal desc_handle1
            nonlocal notify_handle2
            nonlocal desc_handle2
            conn_handle = v["handle"]
            for item in v["services"]:
                print(item["handle"], item["type"], item["data"])
                for gattc in sorted(item["gattcs"].items()):
                    item = gattc[1]
                    print(item["handle"], item["type"], item["data"])
                    t = item["type"]
                    if t == 1.2:
                        conn_handle, def_handle, value_handle, properties, uuid = item["data"]
                        if uuid == _CHARACTERISTIC_UUID:
                            if not notify_handle1:
                                hitting = True
                                notify_handle1 = value_handle
                            elif not notify_handle2:
                                hitting = True
                                notify_handle2 = value_handle
                            else:
                                hitting = False
                                print("Too much, char.", conn_handle, value_handle)
                        else:
                            hitting = False
                    elif t==2:
                        conn_handle, dsc_handle, uuid = item["data"]
                        if hitting and uuid == _DESCRIPTOR_UUID:
                            if notify_handle1 and not desc_handle1:
                                desc_handle1 = dsc_handle
                            elif notify_handle2 and not desc_handle2:
                                desc_handle2 = dsc_handle
                            else:
                                print("Too much, desc.", conn_handle, dsc_handle)

        central.connect(addr_type, addr_bytes, callback=on_conn)

        retry_count = 50
        while central.is_busy():
            time.sleep_ms(100)
            if not_found:
                return
            retry_count = retry_count - 1
            if retry_count < 0:
                print("Timeout.", retry_count)
                return

        print("Connected")

        # notify
        def on_notify(conn_handle, value_handle, notify_data):
            print("on_notify()", conn_handle, value_handle, notify_data, "DOWN" if notify_data[0]==2 else "UP" )

        central.register_notify(conn_handle, notify_handle1, on_notify)
        central.register_notify(conn_handle, notify_handle2, on_notify)

        # enable notify
        v = central.gattc_write_sync(conn_handle, desc_handle1, struct.pack('<h', 1), 1)
        v = central.gattc_write_sync(conn_handle, desc_handle2, struct.pack('<h', 1), 1)

        # connection loop
        while central.is_connected(conn_handle):
            time.sleep_ms(100)
        
        print("Disconnected")

    while True:
        execute()

if __name__ == "__main__":
    demo()
