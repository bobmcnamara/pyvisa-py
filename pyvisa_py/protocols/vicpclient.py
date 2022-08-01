#
# IMPORTANT NOTICE:
#
# This LeCroy VICP (LGPL2.1) code may only be used for Apple internal
# use.  No portion of this code may be included or used in shipping
# software or product.  In addition, before downloading any new/updated
# versions of this code, you must verify that its licensing terms are
# the same, i.e. still under LGPL2.1 (and not LGPL v3).
#
# If you ever need to provide any of the LeCroy VICP code to external
# (non-Apple) vendors or consultants, please email darwin-mgmt
# <darwin-mgt@group.apple.com> first, as that could constitute a
# distribution triggering event.
#
#
# ------------------------------------------------------------------------------
# Summary:    Lightweight VICP client implementation.
#
# Started by: Anthony Cake, June 2003
#
# Python rewrite by: Bob McNamara, December 2016
#
#     Published on SourceForge under LeCroyVICP project, Sept 2003
#
#     This library is free software; you can redistribute it and/or
#     modify it under the terms of the GNU Lesser General Public
#     License as published by the Free Software Foundation; either
#     version 2.1 of the License, or (at your option) any later version.
#
#     This library is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#     Lesser General Public License for more details.
#
#     You should have received a copy of the GNU Lesser General Public
#     License along with this library; if not, write to the Free Software
#     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
# ------------------------------------------------------------------------------
#
# Description:
#
#     This file contains a Client-side implementation of the VICP
#     network communications protocol used to control LeCroy Digital
#     Oscilloscopes (DSOs).
#
# VICP Protocol Description/History:
#
#     The VICP Protocol has been around since 1997/98. It did not
#     change in any way between its conception and June 2003, when
#     a previously reserved field in the header was assigned.  This
#     field, found at byte #2, is now used to allow the client-end
#     of a VICP communication to detect 'out of sync' situations,
#     and therefore allows the IEEE 488.2 'Unread Response' mechanism
#     to be emulated.
#
#     These extensions to the original protocol did not cause a
#     version number change, and are referred to as version 1a. It
#     was decided not to bump the version number to reduce the impact
#     on clients, many of which are looking for a version number of
#     1.  Clients and servers detect protocol version 1a by examining
#     the sequence number field, it should be 0 for version 1 of the
#     protocol (early clients), or non-zero for v1a.
#
# VICP Header is always exactly 8 bytes long:
#
#     Byte   Description
#     -------------------------------------------
#     0      Operation
#     1      Version         1 = version 1
#     2      Sequence Number { 1..255 }, (was unused until June 2003)
#     3      Unused
#     4      Block size, MSB  (not including this header)
#     5      Block size
#     6      Block size
#     7      Block size, LSB
#
#     Byte 0 (Operation) bit definitions:
#
#     Bit    Mnemonic      Purpose
#     ------------------------------------
#     D7     DATA          Data block (D0 indicates with/without EOI)
#     D6     REMOTE        Remote Mode
#     D5     LOCKOUT       Local Lockout (Lockout front panel)
#     D4     CLEAR         Device Clear (if sent with data, clear occurs
#                                        before block is passed to parser)
#     D3     SRQ           SRQ (Device -> PC only)
#     D2     SERIALPOLL    Request a serial poll
#     D1     Reserved      Reserved for future expansion
#     D0     EOI           Block terminated in EOI
#
# Known Limitations:
#
# Outstanding Issues
#
# Dependencies
#     - Uses generic Python sockets, so should run anywhere Python runs.
#
# ------------------------------------------------------------------------------
#

import socket
import struct
import sys
import time

# GPIB status bit vector :
#                    ibsta and wait mask
ERR = 1 << 15  # Error detected                    0x8000
TIMO = 1 << 14  # Timeout                           0x4000
END = 1 << 13  # EOI or EOS detected               0x2000
SRQI = 1 << 12  # SRQ detected by CIC               0x1000
RQS = 1 << 11  # Device needs service              0x0800
SPOLL = 1 << 10  # Board has been serially polled    0x0400
CMPL = 1 << 8  # I/O completed                     0x0100
REM = 1 << 6  # Remote state                      0x0040
CIC = 1 << 5  # Controller-in-Charge              0x0020
ATN = 1 << 4  # Attention asserted                0x0010
TACS = 1 << 3  # Talker active                     0x0008
LACS = 1 << 2  # Listener active                   0x0004
DTAS = 1 << 1  # Device trigger state              0x0002
DCAS = 1 << 0  # Device clear state                0x0001

# GPIB error codes :
#              iberr
EDVR = 0  # System error
ECIC = 1  # Function requires GPIB board to be CIC
ENOL = 2  # Write function detected no Listeners
EADR = 3  # Interface board not addressed correctly
EARG = 4  # Invalid argument to function call
ESAC = 5  # Function requires GPIB board to be SAC
EABO = 6  # I/O operation aborted
ENEB = 7  # Non-existent interface board
EDMA = 8  # Error performing DMA
EOIP = 10  # I/O operation started before previous operation completed
ECAP = 11  # No capability for intended operation
EFSO = 12  # File system operation error
EBUS = 14  # Command error during device call
ESTB = 15  # Serial poll status byte lost
ESRQ = 16  # SRQ remains asserted
ETAB = 20  # The return buffer is full.
ELCK = 21  # Address or board is locked.

SERVER_PORT_NUM = 1861  # port # registered with IANA for lecroy-vicp
HEADER_FORMAT = "!BBBBI"  # format of network header
SMALL_DATA_BUFSIZE = 1 << 20  # below this threshold, header and payload are
# copied into a contiguous buffer before sending.
CONNECT_TIMEOUT_SECS = 2
DEFAULT_TIMEOUT_SECS = 10

# VICP header 'Operation' bits
OPERATION_DATA = 0x80
OPERATION_REMOTE = 0x40
OPERATION_LOCKOUT = 0x20
OPERATION_CLEAR = 0x10
OPERATION_SRQ = 0x08
OPERATION_REQSERIALPOLL = 0x04
OPERATION_UNDEFINED = 0x02  # presumably this is reserved for future use
OPERATION_EOI = 0x01

# Header Version
HEADER_VERSION1 = 0x01

exr_errors = {
    #
    # execution error status register (EXR)
    #
    21: "Permission error. The command cannot be executed in local mode.",
    22: "Environment error. The oscilloscope is not configured to correctly process a command. For instance, the oscilloscope cannot be set to RIS at a slow timebase.",
    23: "Option error. The command applies to an option which has not been installed.",
    24: "Unresolved parsing error.",
    25: "Parameter error. Too many parameters specified.",
    26: "Non-implemented command.",
    27: "Parameter missing. A parameter was expected by the command.",
    30: "Hex data error. A non-hexadecimal character has been detected in a hex data block.",
    31: "Waveform error. The amount of data received does not correspond to descriptor indicators.",
    32: "Waveform descriptor error. An invalid waveform descriptor has been detected.",
    33: "Waveform text error. A corrupted waveform user text has been detected.",
    34: "Waveform time error. Invalid RIS or TRIG time data has been detected.",
    35: "Waveform data error. Invalid waveform data have been detected.",
    36: "Panel setup error. An invalid panel setup data block has been detected.",
    50: "No mass storage present when user attempted to access it.",
    51: "Mass storage not formatted when user attempted to access it.",
    53: "Mass storage was write protected when user attempted to create a file, to delete a file, or to format the device.",
    54: "Bad mass storage detected during formatting.",
    55: "Mass storage root directory full. Cannot add directory.",
    56: "Mass storage full when user attempted to write to it.",
    57: "Mass storage file sequence numbers exhausted (999 reached).",
    58: "Mass storage file not found.",
    59: "Requested directory not found.",
    61: "Mass storage filename not DOS compatible, or illegal filename.",
    62: "Cannot write on mass storage because filename already exists.",
}

# ------------------------------------------------------------------------------
# Exceptions


class VicpException(Exception):
    def __init__(self, err="", note=""):
        self.err = err
        self.note = note
        self.msg = ""

        if err == "":
            self.msg = note
        else:
            self.msg = err
            if note != "":
                self.msg = "%s [%s]" % (self.msg, note)

    def __str__(self):
        return self.msg


class VicpSRQ(VicpException):
    pass


class VicpTimeout(VicpException):
    pass


# ------------------------------------------------------------------------------


class Struct(dict):
    def __init__(self, **kwargs):
        super(Struct, self).__init__(**kwargs)
        self.__dict__ = self


class Enum(object):
    def __init__(self, names):  # or *names, with no .split()
        for number, name in enumerate(names.split()):
            setattr(self, name, number)


READSTATE = Enum("header data")


class Client(object):
    # --------------------------------------------------------------------------
    def __init__(
        self,
        address,
        port=SERVER_PORT_NUM,
        connect_timeout=CONNECT_TIMEOUT_SECS,
        debug=False,
    ):
        self.ipaddr = address  # IP address of the instrument
        self.port = port  # port number
        self._socket = None  # socket object
        self._debug_flag = debug  # if True, log debug messages to stderr
        self._connected_to_scope = False  # connected to scope?
        self._remote_mode = False  # if True, device is in remote mode
        self._local_lockout = False  # if True, device is in local lockout mode
        self._next_sequence_number = 1  # next sequence value
        self._last_sequence_number = None  # last used sequence value
        self._flush_unread_responses = True  # if True, unread responses are flushed
        #  (emulate IEEE 488.2 behaviour)
        self._vicpversion1a_supported = (
            False  # version 1a of the VICP protocol supported
        )
        # (seq. numbers and OOB data)
        self._recv_block = Struct(
            read_state=READSTATE.header,  # current state of read 'state machine'
            flags=None,  # flags byte from received header
            block_size=0,  # total number of payload bytes
            bytes_read=0,  # number of payload bytes received so far
            eoi_terminated=False,  # flag indicating whether current block
            srq_state=False,  # indicates state of SRQ
            srq_state_changed=False,  # indicates a change in SRQ state
            seq_num=None,  # received sequence number
        )

        # finally, connect and configure the default timeout
        self._connect_to_device(connect_timeout)
        self._socket.settimeout(DEFAULT_TIMEOUT_SECS)

    @property
    def timeout(self):
        return self._socket.gettimeout()

    @timeout.setter
    def timeout(self, value):
        self._socket.settimeout(value)

    # --------------------------------------------------------------------------
    def close(self):
        """
        close the connection
        """
        self._debug_log("Disconnecting:")

        # check if connected
        if self._socket is not None:  # check socket handle
            self._debug_log("close %s:" % self._socket)
            self._socket.shutdown(socket.SHUT_RDWR)
            self._socket.close()

        # reset any partial read operation
        self._recv_block.read_state = READSTATE.header
        self._socket = None
        self._connected_to_scope = False
        self._vicpversion1a_supported = False

    # --------------------------------------------------------------------------
    def _connect_to_device(self, timeout=CONNECT_TIMEOUT_SECS):
        """
        connect to a network device

        address is extracted from self.ipaddr (specified during construction of base class)
        """
        # if not yet connected to scope...
        if not self._connected_to_scope:
            # create client's socket
            self._debug_log("Opening Socket:")
            self._socket = socket.create_connection(
                (self.ipaddr, self.port), timeout=timeout
            )
            self._debug_log("Socket opened: '%s'" % self._socket)

            # disable the TCP/IP 'Nagle' algorithm that buffers packets before sending.
            self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            # enable SO_LINGER to allow hard closure of sockets
            l_onoff = 1  # LINGER enabled
            l_linger = 0  # timeout = 0
            enable_linger = struct.pack("ii", l_onoff, l_linger)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, enable_linger)

        self._connected_to_scope = True

    # --------------------------------------------------------------------------
    def _debug_log(self, message):
        if self._debug_flag:
            print(message, file=sys.stderr)

    # --------------------------------------------------------------------------
    # clear the device
    def device_clear(self, force_reconnect=False):
        try:
            self._send_packet(flags=OPERATION_CLEAR)
        except socket.error:
            force_reconnect = True  # failed to send, so try to reconnect

        # if VICP version 1a is not supported (which means that
        # unread response clearing is not supported), then momentarily
        # disconnect from the device in order to clear buffers also
        # do this if a reconnection was forced
        if not self._vicpversion1a_supported or force_reconnect:
            # TODO: remove when 'RebootScope' bug is fixed
            time.sleep(100)
            self.close()
            self._connect_to_device()

    # --------------------------------------------------------------------------
    def serial_poll(self):
        """
        return the serial poll byte.  Use the new Out-Of-Band signaling
        technique if supported, else the original 'in-band' technique.
        """
        if self._vicpversion1a_supported:
            return self._oob_data_request("S")  # 'S' == Serial Poll
        else:
            # request the serial poll using an in-band technique
            self._send_packet(flags=OPERATION_REQSERIALPOLL)

            # read the serial-poll response
            serial_poll_response = self._read_till_end_of_block()
            return serial_poll_response[0]

    # --------------------------------------------------------------------------
    def _oob_data_request(self, request_type):
        """
        out-of band data request
        presently used only for serial polling.
        """
        oob_data_test = struct.pack("!b", request_type)
        self._socket.sendall(oob_data_test, socket.MSG_OOB)
        return self._socket.recv(1, socket.MSG_OOB)

    # --------------------------------------------------------------------------
    def _send_data_to_device(
        self, message="", eoi_termination=False, device_clear=False, serial_poll=False
    ):
        """
        send a block of data to a network device.
        errors raise an exception.
        """
        bytes_to_send = len(message)

        # handle cases where the user didn't read all data from a previous query
        if (
            self._flush_unread_responses
            and self._recv_block.read_state != READSTATE.header
        ):
            # dump the unread partial buffer
            self._receive_flush(
                self._recv_block.block_size - self._recv_block.bytes_read
            )
            self._recv_block.read_state = READSTATE.header

        # send header
        flags = OPERATION_DATA
        if eoi_termination:
            flags |= OPERATION_EOI
        if self._remote_mode:
            flags |= OPERATION_REMOTE
        if device_clear:
            flags |= OPERATION_CLEAR
        if self._local_lockout:
            flags |= OPERATION_LOCKOUT
        if serial_poll:
            flags |= OPERATION_REQSERIALPOLL

        seq_num = self._get_next_sequence_number(eoi_termination)  # sequence number

        header = struct.pack(
            HEADER_FORMAT, flags, HEADER_VERSION1, seq_num, 0, bytes_to_send
        )

        self._debug_log(
            "_send_data_to_device: seq=%d eoi=%d" % (seq_num, eoi_termination)
        )

        if bytes_to_send < SMALL_DATA_BUFSIZE:
            # if the block is relatively small, optimize for network efficiency
            # (use one 'send' command for header + data)
            self._socket.sendall(header + message)
        else:
            # send header and message separately (to avoid copying the data).
            self._socket.sendall(header)
            self._socket.sendall(message)
        bytes_sent = len(message)
        self._debug_log("bytes_sent=%d [%.20s]" % (bytes_sent, message))

    # --------------------------------------------------------------------------
    def _send_packet(self, payload=b"", flags=OPERATION_EOI):
        """
        send a block of data to the device.
        """

        # build the header
        flags |= OPERATION_DATA
        eoi_termination = (flags & OPERATION_EOI) != 0
        seq_num = self._get_next_sequence_number(eoi_termination)  # sequence number
        payload_length = len(payload)
        header = struct.pack(
            HEADER_FORMAT, flags, HEADER_VERSION1, seq_num, 0, payload_length
        )

        self._debug_log("_send_packet: seq=%d eoi=%d" % (seq_num, eoi_termination))
        if payload_length < SMALL_DATA_BUFSIZE:
            # if the block is relatively small, optimize for network efficiency
            # (use one 'send' command for header + data)
            self._socket.sendall(header + payload)
        else:
            # send header and payload separately (to avoid copying the data).
            self._socket.sendall(header)
            self._socket.sendall(payload)
        self._debug_log(
            "_send_packet: bytes_sent=%d [%.20s]"
            % (payload_length, payload.decode("utf-8", errors="ignore"))
        )

    # --------------------------------------------------------------------------
    def _get_next_sequence_number(self, eoi_termination):
        """
        Return the next sequence number in the range 1..255
        (Note that zero is omitted intentionally)

        used to synchronize write/read operations, attempting to
        emulate the 488.2 'discard unread response' behaviour
        """

        # we'll return the current sequence number
        self._last_sequence_number = self._next_sequence_number

        # which then gets incremented if this block is EOI terminated
        if eoi_termination:
            self._next_sequence_number += 1
            if self._next_sequence_number >= 256:
                self._next_sequence_number = 1

        return self._last_sequence_number

    # --------------------------------------------------------------------------
    def send(self, cmdstr):
        try:
            self._send_packet(cmdstr)
        except socket.error as exc:
            if str(exc) == "timed out":
                raise VicpTimeout(str(exc))
            else:
                raise

    # --------------------------------------------------------------------------
    def receive(self, buf_len=None):
        try:
            return self._read_till_whenever(buf_len, stop_at_eob=False)
        except socket.error as exc:
            if str(exc) == "timed out":
                raise VicpTimeout(str(exc))
            else:
                raise

    # --------------------------------------------------------------------------
    def recv_block(self, buf_len=None):
        try:
            return self._read_till_whenever(buf_len, stop_at_eob=True)
        except socket.error as exc:
            if str(exc) == "timed out":
                raise VicpTimeout(str(exc))
            else:
                raise

    # --------------------------------------------------------------------------
    # dump data until the next header is found
    def _receive_flush(self, bytes_to_dump):
        """
        flush exactly 'bytes_to_dump' bytes from socket.
        received data is thrown away and nothing is returned
        """
        error_message = (
            "_receive_flush: unread response, dumping %d bytes\n" % bytes_to_dump
        )
        self._debug_log(error_message)

        self._receive_exact(bytes_to_dump)

    # --------------------------------------------------------------------------
    def _receive_exact(self, recv_len):
        """
        receive exactly 'recv_len' bytes from socket.
        returns a bytearray containing the received data.
        """
        recv_buffer = bytearray(recv_len)
        self._receive_exact_into(recv_buffer)
        return recv_buffer

    # --------------------------------------------------------------------------
    def _receive_exact_into(self, recv_buffer):
        """
        receive data from socket to exactly fill 'recv_buffer'.
        """
        view = memoryview(recv_buffer)
        recv_len = len(recv_buffer)
        bytes_recvd = 0

        while bytes_recvd < recv_len:
            request_size = recv_len - bytes_recvd
            data_len = self._socket.recv_into(view, request_size)
            bytes_recvd += data_len
            view = view[data_len:]

        if bytes_recvd > recv_len:
            raise MemoryError("socket.recv_into scribbled past end of recv_buffer")

    # --------------------------------------------------------------------------
    def _read_till_whenever(self, buf_len, stop_at_eob):
        # use default length if None given
        req_len = buf_len or 4096

        buf_list = list()

        while True:
            # loop until one of the following occurs:
            #     1. we have read the requested number of bytes (buf_len)
            #     2. stop_at_eob is True and we're at the end of a block
            #     3. src_block.eoi_terminated is True
            recv_buffer = self._read_from_device(req_len, stop_at_eob=stop_at_eob)
            nbytes = len(recv_buffer)
            if nbytes < req_len:
                # truncate the buffer to the actual number of bytes received
                recv_buffer = recv_buffer[:nbytes]
            buf_list.append(recv_buffer)

            if buf_len is not None:
                # we've either read the requested number of bytes or hit a stop
                # condition.  in either case, we're done.  exit the loop.
                break

            if self._recv_block.read_state == READSTATE.header:
                # we're at the end of a data block...
                if stop_at_eob or self._recv_block.eoi_terminated:
                    break

        if len(buf_list) > 1:
            # multiple buffers, they need to be concatenated
            result = b"".join(buf_list)
        else:
            # only one buffer; it's our result
            result = buf_list[0]

        return result

    # --------------------------------------------------------------------------
    def _read_from_device(self, req_len, stop_at_eob=False, return_after_srq=False):
        """
        read block of data from a network device
        """
        reply_buf = bytearray(req_len)
        user_buffer_size_bytes = len(reply_buf)
        user_buffer_bytes_written = 0  # number of bytes placed in user buffer
        view = memoryview(reply_buf)

        while True:
            # loop until one of the following occurs:
            #     1. srq_state_changed is True and return_after_srq is True
            #     2. stop_at_eob is True and we're at the end of a block
            #     3. src_block.eoi_terminated is True and we're at the end of a block
            #     4. we've read as many bytes as requested
            if self._recv_block.read_state == READSTATE.header:
                try:
                    self._read_next_header()
                except VicpSRQ:
                    # if we saw SRQ come (or go), and the user requests that we
                    # return immediately after seeing the change, then return
                    if return_after_srq:
                        break

            if self._recv_block.read_state == READSTATE.data:
                # fill the user-supplied buffer (but no more than
                # src_bytes_available bytes)
                user_buffer_free_space = (
                    user_buffer_size_bytes - user_buffer_bytes_written
                )
                src_bytes_available = (
                    self._recv_block.block_size - self._recv_block.bytes_read
                )

                recv_len = min(user_buffer_free_space, src_bytes_available)

                self._receive_exact_into(view[:recv_len])

                self._recv_block.bytes_read += recv_len
                user_buffer_bytes_written += recv_len
                view = view[recv_len:]

                if self._recv_block.bytes_read >= self._recv_block.block_size:
                    # we have finished reading the contents of this
                    # header-prefixed block.  go back to the state where we can
                    # watch for the next block.
                    self._recv_block.read_state = READSTATE.header

                    if self._recv_block.eoi_terminated:
                        break

                    if stop_at_eob:
                        # stop at end of block
                        break

                if user_buffer_bytes_written >= user_buffer_size_bytes:
                    # the user's buffer has been filled.  get out of the loop.
                    break

        if user_buffer_bytes_written < req_len:
            reply_buf = reply_buf[:user_buffer_bytes_written]

        self._debug_log(
            "_read_from_device: returning %d bytes" % user_buffer_bytes_written
        )

        return reply_buf

    # --------------------------------------------------------------------------
    def _read_srq_packet(self):
        """
        read the SRQ packet, process it, and toss it.
        errors will raise an exception.
        """
        assert self._recv_block.read_state == READSTATE.data
        srq_packet = self._receive_exact(self._recv_block.block_size)
        self._recv_block.read_state = READSTATE.header
        # '1' = asserted, '0' = deasserted
        self._recv_block.srq_state = srq_packet[0] == "1"

    # --------------------------------------------------------------------------
    def _read_next_header(self):
        """
        read the next header, flushing any unread (old) responses along the way.
        errors will raise an exception.
        """
        while self._recv_block.read_state == READSTATE.header:
            self._read_one_header()

            # header was successfully read
            status_message = " ".join(
                [
                    "Read Header:",
                    "block_size=%d," % self._recv_block.block_size,
                    "EOI=%d," % self._recv_block.eoi_terminated,
                    "SRQ state changed=%d," % self._recv_block.srq_state_changed,
                    "seq_num=%d," % self._recv_block.seq_num,
                ]
            )
            self._debug_log(status_message)

            if self._recv_block.srq_state_changed:
                self._read_srq_packet()
                raise VicpSRQ
            elif self._recv_block.seq_num == 0:
                # we're talking to a scope running pre-June 2003 code that
                # didn't support sequence numbering, and therefore we don't
                # know when to dump data.
                self._vicpversion1a_supported = False
            else:
                # version '1a' of the VICP protocol is in use, which in
                # addition to sequence numbering supports the use of
                # out-of-band signaling.
                self._vicpversion1a_supported = True

                # if we're flushing unread responses, and this header contains
                # an unexpected sequence number (older than the current one),
                # then dump this block and go around again.
                raw_delta = self._last_sequence_number - self._recv_block.seq_num
                seq_num_delta = raw_delta + 255 if raw_delta < -128 else raw_delta
                if self._flush_unread_responses and seq_num_delta > 0:
                    self._receive_flush(self._recv_block.block_size)
                    self._recv_block.read_state = READSTATE.header

    # --------------------------------------------------------------------------

    def _read_one_header(self):
        """
        read one header
        assumes that READSTATE is 'header'.
        errors will raise an exception
        """
        # receive the scope's response, header first
        header_size = struct.calcsize(HEADER_FORMAT)
        try:
            header_buf = self._receive_exact(header_size)
        except socket.error as exc:
            if str(exc) == "timed out":
                # a timeout is usually due to a bad query
                # ...no need to disconnect
                raise VicpTimeout
            else:
                # something more serious than a timeout. since we are
                # out of sync, need to close & reopen the socket
                self.close()
                self._connect_to_device()
                raise VicpException(str(exc))

        flags, version, seq_num, _, block_size = struct.unpack(
            HEADER_FORMAT, header_buf
        )

        # check the integrity of the header
        if not ((flags & OPERATION_DATA) and (version == HEADER_VERSION1)):
            error_message_format = (
                "Invalid Header! (header received" + " %02x" * header_size + ")"
            )
            error_message = error_message_format % tuple(header_buf)
            self._debug_log(error_message)

            # error state, cannot recognise header. since we are
            # out of sync, need to close & reopen the socket
            self.close()
            self._connect_to_device()
            raise VicpException(error_message)

        # decode the bits in 'flags'
        self._recv_block.flags = flags
        self._recv_block.srq_state_changed = flags & OPERATION_SRQ
        self._recv_block.eoi_terminated = flags & OPERATION_EOI
        # self._recv_block.data = flags & OPERATION_DATA
        # self._recv_block.remote = flags & OPERATION_REMOTE
        # self._recv_block.lockout = flags & OPERATION_LOCKOUT
        # self._recv_block.clear = flags & OPERATION_CLEAR
        # self._recv_block.req_serial_poll = flags & OPERATION_REQSERIALPOLL

        self._recv_block.block_size = block_size
        self._recv_block.seq_num = seq_num
        self._recv_block.bytes_read = 0

        self._recv_block.read_state = READSTATE.data
