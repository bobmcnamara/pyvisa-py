import socket
import select
import time

from typing import Any, Tuple

from pyvisa import constants, attributes
from pyvisa.constants import ResourceAttribute, StatusCode, BufferOperation

from pyvisa_py.sessions import Session, UnknownAttribute
from pyvisa_py import tcpip
from pyvisa_py import common

@Session.register(constants.InterfaceType.prlgx_tcpip, "INSTR")
class PrologixTCPIPInstrSession(Session):

    def after_parsing(self) -> None:
        print(f"pyvisa_py.prologix.PrologixTCPIPInstrSession.after_parsing: {self=}, {self.parsed=}")

        ret_status = self._connect()
        if ret_status != StatusCode.success:
            self.close()
            raise Exception("could not connect: {0}".format(str(ret_status)))

        self.max_recv_size = 4096
        # This buffer is used to store the bytes that appeared after
        # termination char
        self._pending_buffer = bytearray()

################################################

        # initialize the constant attributes
        #self.attrs[ResourceAttribute.dma_allow_enabled] = constants.VI_FALSE
        #self.attrs[ResourceAttribute.file_append_enabled] = constants.VI_FALSE
        #self.attrs[ResourceAttribute.interface_instrument_name] = "TCPIP0 (HiSLIP)"
        #self.attrs[ResourceAttribute.interface_number] = 0
        #self.attrs[ResourceAttribute.io_prot] = constants.VI_PROT_NORMAL
        #self.attrs[ResourceAttribute.read_buffer_operation_mode] = (constants.VI_FLUSH_DISABLE)
        #self.attrs[ResourceAttribute.resource_lock_state] = constants.VI_NO_LOCK
        #self.attrs[ResourceAttribute.send_end_enabled] = constants.VI_TRUE
        #self.attrs[ResourceAttribute.suppress_end_enabled] = constants.VI_FALSE
        self.attrs[ResourceAttribute.tcpip_address] = self.parsed.host_address
        #self.attrs[ResourceAttribute.tcpip_hostname] = self.parsed.host_address
        self.attrs[ResourceAttribute.tcpip_port] = self.parsed.port
        self.attrs[ResourceAttribute.termchar] = ord("\n")
        self.attrs[ResourceAttribute.termchar_enabled] = constants.VI_TRUE
        #self.attrs[ResourceAttribute.write_buffer_operation_mode] = (constants.VI_FLUSH_WHEN_FULL)

################################################

        self.attrs[ResourceAttribute.interface_number] = self.parsed.board
        self.attrs[ResourceAttribute.tcpip_nodelay] = (
            self._get_tcpip_nodelay,
            self._set_attribute,
        )
        self.attrs[ResourceAttribute.tcpip_hostname] = ""
        self.attrs[ResourceAttribute.tcpip_keepalive] = (
            self._get_tcpip_keepalive,
            self._set_tcpip_keepalive,
        )
        # to use default as ni visa driver (NI-VISA 15.0)
        self.attrs[ResourceAttribute.suppress_end_enabled] = True

        #for name in ("TERMCHAR", "TERMCHAR_EN"):
        #    attribute = getattr(constants, "VI_ATTR_" + name)
        #    self.attrs[attribute] = attributes.AttributesByID[attribute].default

    def _connect(self) -> StatusCode:
        timeout = self.open_timeout / 1000.0 if self.open_timeout else 10.0
        try:
            self.interface = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.interface.setblocking(False)
            self.interface.connect_ex((self.parsed.host_address, int(self.parsed.port)))
        except Exception as e:
            raise Exception("could not connect: {0}".format(str(e)))
        finally:
            self.interface.setblocking(True)

        # minimum is in interval 100 - 500ms based on timeout
        min_select_timeout = max(min(timeout / 10.0, 0.5), 0.1)
        # initial 'select_timout' is half of timeout or max 2 secs
        # (max blocking time). min is from 'min_select_timeout'
        select_timout = max(min(timeout / 2.0, 2.0), min_select_timeout)
        # time, when loop shall finish
        finish_time = time.time() + timeout
        while True:
            # use select to wait for socket ready, max `select_timout` seconds
            r, w, x = select.select(
                [self.interface], [self.interface], [], select_timout
            )
            if self.interface in r or self.interface in w:
                return StatusCode.success

            if time.time() >= finish_time:
                # reached timeout
                return StatusCode.error_timeout

            # `select_timout` decreased to 50% of previous or
            # min_select_timeout
            select_timout = max(select_timout / 2.0, min_select_timeout)

    def close(self) -> StatusCode:
        if self.interface:
            self.interface.close()
        self.interface = None
        return StatusCode.success

    def read(self, count: int) -> Tuple[bytes, StatusCode]:
        """Reads data from device or interface synchronously.

        Corresponds to viRead function of the VISA library.

         Parameters
        -----------
        count : int
            Number of bytes to be read.

        Returns
        -------
        bytes
            Data read from the device
        StatusCode
            Return value of the library call.

        """
        if count < self.max_recv_size:
            chunk_length = count
        else:
            chunk_length = self.max_recv_size

        term_char, _ = self.get_attribute(ResourceAttribute.termchar)
        term_byte = common.int_to_byte(term_char) if term_char is not None else b""
        term_char_en, _ = self.get_attribute(ResourceAttribute.termchar_enabled)
        suppress_end_en, _ = self.get_attribute(ResourceAttribute.suppress_end_enabled)

        print(f"**** pyvisa_py.PrologixTCPIPInstrSession.read: {term_char=}, {term_byte=}, {term_char_en=}, {suppress_end_en=}")

        read_fun = self.interface.recv

        # minimum is in interval 1 - 100ms based on timeout, 1sec if no timeout
        # defined
        min_select_timeout = (
            1 if self.timeout is None else max(min(self.timeout / 100.0, 0.1), 0.001)
        )
        # initial 'select_timout' is half of timeout or max 2 secs
        # (max blocking time). min is from 'min_select_timeout'
        select_timout = (
            2.0
            if self.timeout is None
            else max(min(self.timeout / 2.0, 2.0), min_select_timeout)
        )
        # time, when loop shall finish, None means never ending story if no
        # data arrives
        finish_time = None if self.timeout is None else (time.time() + self.timeout)
        while True:
            # check, if we have any data received (from pending buffer or
            # further reading)
            if term_char_en and term_byte in self._pending_buffer:
                term_byte_index = self._pending_buffer.index(term_byte) + 1
                if term_byte_index > count:
                    term_byte_index = count
                    status = StatusCode.success_max_count_read
                else:
                    status = StatusCode.success_termination_character_read
                out = bytes(self._pending_buffer[:term_byte_index])
                self._pending_buffer = self._pending_buffer[term_byte_index:]
                return out, status

            if len(self._pending_buffer) >= count:
                out = bytes(self._pending_buffer[:count])
                self._pending_buffer = self._pending_buffer[count:]
                return out, StatusCode.success_max_count_read

            # use select to wait for read ready, max `select_timout` seconds
            r, w, x = select.select([self.interface], [], [], select_timout)

            read_data = b""
            if self.interface in r:
                read_data = read_fun(chunk_length)
                self._pending_buffer.extend(read_data)

            if not read_data:
                # can't read chunk or timeout
                if self._pending_buffer and not suppress_end_en:
                    # we have some data without termchar but no further data
                    # expected
                    out = bytes(self._pending_buffer[:count])
                    self._pending_buffer = self._pending_buffer[count:]
                    return out, StatusCode.success

                if finish_time and time.time() >= finish_time:
                    # reached timeout
                    out = bytes(self._pending_buffer[:count])
                    self._pending_buffer = self._pending_buffer[count:]
                    return out, StatusCode.error_timeout

                # `select_timout` decreased to 50% of previous or
                # min_select_timeout
                select_timout = max(select_timout / 2.0, min_select_timeout)

    def write(self, data: bytes) -> Tuple[int, StatusCode]:
        """Writes data to device or interface synchronously.

        Corresponds to viWrite function of the VISA library.

        Parameters
        ----------
        data : bytes
            Data to be written.

        Returns
        -------
        int
            Number of bytes actually transferred
        StatusCode
            Return value of the library call.

        """
        chunk_size = 4096

        num = sz = len(data)

        offset = 0

        while num > 0:
            block = data[offset : min(offset + chunk_size, sz)]

            try:
                # use select to wait for write ready
                select.select([], [self.interface], [])
                size = self.interface.send(block)
            except socket.timeout:
                return offset, StatusCode.error_io

            if size < len(block):
                return offset, StatusCode.error_io

            offset += size
            num -= size

        return offset, StatusCode.success

    def clear(self) -> StatusCode:
        """Clears a device.

        Corresponds to viClear function of the VISA library.

        """
        self._pending_buffer.clear()
        while True:
            r, w, x = select.select([self.interface], [], [], 0.1)
            if not r:
                break
            r[0].recv(4096)

        return StatusCode.success

    def flush(self, mask: BufferOperation) -> StatusCode:
        """Flush the specified buffers.
        Corresponds to viFlush function of the VISA library.
        Parameters
        ----------
        mask : constants.BufferOperation
            Specifies the action to be taken with flushing the buffer.
            The values can be combined using the | operator. However multiple
            operations on a single buffer cannot be combined.
        Returns
        -------
        constants.StatusCode
            Return value of the library call.
        """
        if mask & BufferOperation.discard_read_buffer:
            self.clear()
        if (
            mask & BufferOperation.discard_read_buffer_no_io
            or mask & BufferOperation.discard_receive_buffer
            or mask & BufferOperation.discard_receive_buffer2
        ):
            self._pending_buffer.clear()
        if (
            mask & BufferOperation.flush_write_buffer
            or mask & BufferOperation.flush_transmit_buffer
            or mask & BufferOperation.discard_write_buffer
            or mask & BufferOperation.discard_transmit_buffer
        ):
            pass

        return StatusCode.success

    def _get_tcpip_nodelay(
        self, attribute: ResourceAttribute
    ) -> Tuple[constants.VisaBoolean, StatusCode]:
        if self.interface:
            value = self.interface.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY)
            return (
                constants.VisaBoolean.true
                if value == 1
                else constants.VisaBoolean.false,
                StatusCode.success,
            )
        return constants.VisaBoolean.false, StatusCode.error_nonsupported_attribute

    def _set_tcpip_nodelay(
        self, attribute: ResourceAttribute, attribute_state: bool
    ) -> StatusCode:
        if self.interface:
            self.interface.setsockopt(
                socket.IPPROTO_TCP, socket.TCP_NODELAY, 1 if attribute_state else 0
            )
            return StatusCode.success
        return StatusCode.error_nonsupported_attribute

    def _get_tcpip_keepalive(
        self, attribute: ResourceAttribute
    ) -> Tuple[constants.VisaBoolean, StatusCode]:
        if self.interface:
            value = self.interface.getsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE)
            return (
                constants.VisaBoolean.true
                if value == 1
                else constants.VisaBoolean.false,
                StatusCode.success,
            )
        return constants.VisaBoolean.false, StatusCode.error_nonsupported_attribute

    def _set_tcpip_keepalive(
        self, attribute: ResourceAttribute, attribute_state: bool
    ) -> StatusCode:
        if self.interface:
            self.interface.setsockopt(
                socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1 if attribute_state else 0
            )
            self.interface.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 60)
            self.interface.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 60)
            self.interface.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 5)
            return StatusCode.success
        return StatusCode.error_nonsupported_attribute

    def _get_attribute(self, attribute: ResourceAttribute) -> Tuple[Any, StatusCode]:
        """Get the value for a given VISA attribute for this session.

        Use to implement custom logic for attributes.

        Parameters
        ----------
        attribute : ResourceAttribute
            Attribute for which the state query is made

        Returns
        -------
        Any
            State of the queried attribute for a specified resource
        StatusCode
            Return value of the library call.

        """
        print(f"prologix._get_attribute({attribute=})")
        if attribute in self.attrs:
            return self.attrs[attribute], StatusCode.success

        raise UnknownAttribute(attribute)

    def _set_attribute(
        self, attribute: ResourceAttribute, attribute_state: Any
    ) -> StatusCode:
        """Sets the state of an attribute.

        Corresponds to viSetAttribute function of the VISA library.

        Parameters
        ----------
        attribute : constants.ResourceAttribute
            Attribute for which the state is to be modified. (Attributes.*)
        attribute_state : Any
            The state of the attribute to be set for the specified object.

        Returns
        -------
        StatusCode
            Return value of the library call.

        """
        raise UnknownAttribute(attribute)
