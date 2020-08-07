import sys
import serial
import serial_asyncio 
import asyncio
import typing
import struct
import dataclasses
import ctypes
import queue
# import pynrfjprog as nrf
import traceback


def log_error(msg):
    print(f"Error: {msg}", file=sys.stderr)

def log_bytes(msg, data):
    print("{}: [{}]".format(msg, ", ".join(f"{b:X}" for b in data)))


class CobsDecodeError(ValueError):
    pass


class CobsCoder:
    STATE_WAIT = 0
    STATE_DECODE = 1
    STATE_INVALID = 2

    DELIMITER = 0
    MAX_BYTES = 255
    OVERHEAD_BYTES = 2
    MAX_DATA_BYTES = MAX_BYTES - OVERHEAD_BYTES

    def __init__(self):
        self.__decode_next_delimiter = 0
        self.__decode_data = bytearray()
        self.__decode_state = CobsCoder.STATE_WAIT

    def encode(self, data : typing.ByteString):
        for in_i in range(0, len(data), CobsCoder.MAX_DATA_BYTES):
            decoded_length = min(len(data) - in_i, CobsCoder.MAX_DATA_BYTES)
            encoded_length = decoded_length + CobsCoder.OVERHEAD_BYTES
            encoded = bytearray(encoded_length)
            last_delimiter = 0
            for j, b in enumerate(data[in_i:in_i + encoded_length]):
                out_i = j - in_i + 1
                if b == CobsCoder.DELIMITER:
                    encoded[last_delimiter] = out_i 
                    last_delimiter = out_i 
                else:
                    encoded[out_i] = b
            encoded[last_delimiter] = encoded_length - 1
            encoded[encoded_length - 1] = CobsCoder.DELIMITER
            yield encoded
                    
    def decode(self, data : typing.ByteString):
        for b in data:
            if self.__decode_state == CobsCoder.STATE_WAIT:
                if b != CobsCoder.DELIMITER:
                    self.__decode_data.clear()
                    self.__decode_next_delimiter = int(b)
                    self.__decode_state = CobsCoder.STATE_DECODE
            elif self.__decode_state == CobsCoder.STATE_DECODE:
                if b != CobsCoder.DELIMITER:
                    if self.__decode_next_delimiter - 1 == len(self.__decode_data):
                        self.__decode_data.append(CobsCoder.DELIMITER)
                        if b > len(self.__decode_data):
                            self.__decode_next_delimiter = int(b)
                        else:
                            self.__decode_state = CobsCoder.STATE_INVALID
                    else:
                        self.__decode_data.append(b)
                else:
                    self.__decode_state = CobsCoder.STATE_WAIT
                    yield self.__decode_data
                    continue
                
                if len(self.__decode_data) >= CobsCoder.MAX_DATA_BYTES:
                    self.__decode_state = CobsCoder.STATE_INVALID

            else:
                if b == CobsCoder.DELIMITER:
                    self.__decode_state = CobsCoder.STATE_WAIT

    def decode_reset(self):
        self.__decode_next_delimiter = 0
        self.__decode_data = bytearray()
        self.__decode_state = CobsCoder.STATE_WAIT



@dataclasses.dataclass
class UartDfuStruct:
    def __bytes__(self):
        field_names = [f.name for f in dataclasses.fields(self)]
        fmt = getattr(self, "FMT")
        for n in field_names:
            if n.endswith("_data"):
                fmt = fmt.format(len(getattr(self, n)))
                break
        field_values = [getattr(self, n) for n in field_names]
        for i, v in enumerate(field_values):
            if isinstance(v, ctypes.Structure) or isinstance(v, ctypes.Union):
                field_values[i] = bytes(v)
        return struct.pack(fmt, *field_values)

    @staticmethod
    def from_bytes(cls, data : typing.ByteString):
        fmt = getattr(cls, "FMT")
        fixed_size = struct.calcsize(fmt.format(0))
        variable_size = len(data) - fixed_size
        fmt = fmt.format(variable_size)
        field_values = list(struct.unpack(fmt, data))
        field_types = [f.type for f in dataclasses.fields(cls)]
        for i, (v, t) in enumerate(zip(field_values, field_types)):
            if issubclass(t, ctypes.Structure) or issubclass(t, ctypes.Union):
                field_values[i] = t.from_buffer_copy(bytes(v)) 
        return cls(*field_values)


class UartDfuHeader(ctypes.LittleEndianStructure):
    _fields_ = [
        ("opcode", ctypes.c_uint8, 3),
        ("status", ctypes.c_uint8, 1),
        ("rfu", ctypes.c_uint8, 4)
    ]

    def __init__(self, opcode, status):
        super(UartDfuHeader, self).__init__(opcode=opcode, status=status, rfu=0)
    
    def __repr__(self):
        return f"UartDfuHeader(opcode=0x{self.opcode:X}, status={self.status})"



@dataclasses.dataclass
class UartDfuInit(UartDfuStruct):
    OPCODE = 0x00
    FMT = "<1sI"

    header : UartDfuHeader = UartDfuHeader(OPCODE, 0) 
    file_size : int = 0


@dataclasses.dataclass
class UartDfuWriteh(UartDfuStruct):
    OPCODE = 0x01
    FMT = "<1sI"

    header : UartDfuHeader = UartDfuHeader(OPCODE, 0)
    fragment_total_size : int = 0


@dataclasses.dataclass
class UartDfuWritec(UartDfuStruct):
    OPCODE = 0x02 
    FMT = "<1s{}s"
    MAX_PAYLOAD_SIZE = CobsCoder.MAX_DATA_BYTES - 1

    header : UartDfuHeader = UartDfuHeader(OPCODE, 0)
    fragment_data : bytearray = bytearray() 


@dataclasses.dataclass
class UartDfuOffset(UartDfuStruct):
    OPCODE = 0x03
    FMT = "<1sI"
    
    header : UartDfuHeader = UartDfuHeader(OPCODE, 0)
    padding : int = 0


class UartDfuDoneArgs(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("padding1", ctypes.c_uint8),
        ("padding2", ctypes.c_uint8),
        ("padding3", ctypes.c_uint8),
        ("padding4", ctypes.c_uint8, 7),
        ("success", ctypes.c_uint8, 1),
    ]

    def __init__(self, success):
        super(UartDfuDoneArgs, self).__init__(padding1=0,
                                              padding2=0,
                                              padding3=0,
                                              padding4=0,
                                              success=success)
    
    def __repr__(self):
        return f"UartDfuDoneArgs(success={self.success})"


@dataclasses.dataclass
class UartDfuDone(UartDfuStruct):
    OPCODE = 0x04
    FMT = "<1s4s"

    header : UartDfuHeader = UartDfuHeader(OPCODE, 0)
    args : UartDfuDoneArgs = UartDfuDoneArgs(0)


class UartDfuStatusArgs(ctypes.Union):
    _fields_ = [
        ("status", ctypes.c_int32),
        ("offset", ctypes.c_uint32)
    ]

    def __repr__(self):
        return f"UartDfuStatusArgs(status/offset={self.status})"


@dataclasses.dataclass
class UartDfuStatus(UartDfuStruct):
    FMT = "<1s4s"
    
    header : UartDfuHeader
    args : UartDfuStatusArgs


class PduParseError(ValueError):
    pass


class UartDfuParser:
    OPCODE_MAP = {
        structure.OPCODE: structure for structure in [
            UartDfuInit,
            UartDfuWriteh,
            UartDfuWritec,
            UartDfuOffset,
            UartDfuDone
        ]
    }

    @staticmethod
    def parse_bytes(data : typing.ByteString):
        if len(data) == 0:
            raise PduParseError("Unable to parse empty PDU")
        try:
            header = UartDfuHeader.from_buffer_copy(data[:ctypes.sizeof(UartDfuHeader)])
            if header.opcode not in UartDfuParser.OPCODE_MAP:
                raise PduParseError(f"Unknown PDU opcode: {header.opcode}")
            if not header.status:
                message_class = UartDfuParser.OPCODE_MAP.get(header.opcode)
                return UartDfuStruct.from_bytes(message_class, data)
            else:
                return UartDfuStruct.from_bytes(UartDfuStatus, data)
        except Exception as e:
            raise PduParseError(str(e))


"""
def test_cc():
    d = [[2,8],[3],[4,5],[6],[0]]
    c = CobsCoder()
    for i, vs in enumerate(d):
        print(f"Iteration {i}")
        for m in c.decode(bytes(vs)):
            print(f"Got: {m}")
"""

class UartDfuProtocolError(ValueError):
    @staticmethod
    def bad_status(reply):
        return UartDfuProtocolError(f"Bad status: {reply.args.status}")

    @staticmethod
    def unexpected_reply(reply):
        return UartDfuProtocolError(f"Unexpected reply: {type(reply).__name__}")

    @staticmethod
    def receive_timeout(timeout):
        return UartDfuProtocolError(f"Receive timeout after {timeout} seconds.")


class UartDfuProtocolParams:
    def __init__(self, loop=None):
        if loop is not None:
            self.enter_future = loop.create_future()
            self.exit_future = loop.create_future()
        else:
            self.enter_future = asyncio.Future()
            self.exit_future = asyncio.Future()
        self.queue = asyncio.Queue() 


class UartDfuProtocol(asyncio.Protocol):
    def __init__(self, protocol_params):
        self.coder = CobsCoder()
        self.protocol_params = protocol_params

    def connection_made(self, transport):
        self.transport = transport
        self.protocol_params.enter_future.set_result(True)

    def data_received(self, data):
        log_bytes(f"RX (raw, len={len(data)})", data)
        for decoded_pdu in self.coder.decode(data):
            log_bytes(f"RX (decoded, len={len(decoded_pdu)})", decoded_pdu)
            try:
                pdu = UartDfuParser.parse_bytes(decoded_pdu)
                self.protocol_params.queue.put_nowait(pdu)
            except PduParseError as e:
                log_error(e)
            except asyncio.QueueFull:
                log_error("Input queue full - dropped PDU.")

    def data_send(self, data):
        log_bytes(f"TX (raw, len={len(data)})", data)
        for encoded_pdu in self.coder.encode(data):
            log_bytes(f"TX (encoded, len={len(encoded_pdu)})", encoded_pdu)
            self.transport.write(encoded_pdu)

    def connection_lost(self, exception):
        if exception is not None:
            log_error(exception)
        self.protocol_params.exit_future.set_result(True)


async def create_serial_connection(loop, protocol_factory, port, baudrate, *args, **kwargs):
    ser = serial.Serial(port, baudrate=9600, rtscts=True, exclusive=True, *args, **kwargs)
    ser.baudrate = baudrate
    protocol = protocol_factory()
    transport = serial_asyncio.SerialTransport(loop, protocol, ser)
    transport.resume_reading()
    return (transport, protocol)


class UartDfuClient:
    SERIAL_TIMEOUT = 2.0
    RECEIVE_TIMEOUT = 5.0

    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate

    def stop(self):
        pass

    def update(self, image_data, fragment_size=1024):
        asyncio.run(self.__serial_run_concurrently(self.__update, image_data, fragment_size))
 
    async def __serial_run_concurrently(self, coro, *args, **kwargs):
        loop = asyncio.get_event_loop()
        protocol_params = UartDfuProtocolParams(loop)
        serial_coro = create_serial_connection(
            loop,
            lambda: UartDfuProtocol(protocol_params),
            port=self.port,
            baudrate=self.baudrate)
        transport, protocol = await asyncio.wait_for(serial_coro, self.SERIAL_TIMEOUT)
        send = self.__send_function(protocol)
        recv = self.__receive_function(protocol_params.queue)
        print(f"RTS: {transport.serial.rts}")
        try:
            print(f"Connected to {self.port}@{self.baudrate}")
            await coro(send, recv, *args, **kwargs)
        finally:
            transport.close()

    async def __update(self, send, recv, image_data, fragment_size):
        send(UartDfuInit(file_size=len(image_data)))
        await recv(self.RECEIVE_TIMEOUT)
        # await asyncio.sleep(1)
        for f_start in range(0, len(image_data), fragment_size):
            current_fragment_size = min(len(image_data) - f_start, fragment_size)
            f_end = f_start + current_fragment_size
            print(f"Uploading fragment starting at {f_start} with size {current_fragment_size}.")
            send(UartDfuWriteh(fragment_total_size=current_fragment_size))
            await recv(self.RECEIVE_TIMEOUT)
            # await asyncio.sleep(1)
            for s_start in range(f_start, f_end, UartDfuWritec.MAX_PAYLOAD_SIZE):
                segment_size = min(f_end - s_start, UartDfuWritec.MAX_PAYLOAD_SIZE)
                segment_data = bytearray(image_data[s_start:s_start + segment_size])
                send(UartDfuWritec(fragment_data=segment_data))
                # await asyncio.sleep(1)
            await recv(self.RECEIVE_TIMEOUT)
            # await asyncio.sleep(1)
        send(UartDfuDone(args=UartDfuDoneArgs(success=1)))
        await recv(self.RECEIVE_TIMEOUT)
        # await asyncio.sleep(1)
        print(f"Update done: sent {len(image_data)} bytes.")
        """
        except Exception as e:
            print(f"Error: {e}")
            message = UartDfuDone(success=False)
            self.__send(message, True)
            reply = self.__receive_status(message, 5.0)
            raise e
        """

    def __send_function(self, protocol):
        def f(message):
            protocol.data_send(bytes(message))
            print(f"--> {message}.")
        return f

    def __receive_function(self, input_queue):
        async def f(timeout, check_status=True):
            try:
                reply = await asyncio.wait_for(input_queue.get(), timeout)
            except asyncio.TimeoutError:
                raise UartDfuProtocolError.receive_timeout(timeout)
            print(f"<-- {reply}")
            if not isinstance(reply, UartDfuStatus):
                raise UartDfuProtocolError.unexpected_reply(reply)
            if check_status and reply.args.status != 0:
                raise UartDfuProtocolError.bad_status(reply)
            return reply
        return f

"""
def board_reset(port):
    with nrf.LowLevel.API(nrf.LowLevel.DeviceFamily.UNKNOWN) as api:
        serial_numbers = api.enum_emu_snr()
        for snr in serial_numbers:
            ports = api.enum_emu_com_ports(snr)
            if port in ports:
                serial_number = snr
                break
        else:
            raise ValueError(f"Serial number for port {port} not found.")
        api.connect_to_emu_with_snr(serial_number)
        api.sys_reset()
"""