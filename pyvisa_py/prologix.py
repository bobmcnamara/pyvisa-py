from typing import Optional, List, Tuple, Any

from pyvisa import constants, rname
from pyvisa.constants import BufferOperation, ResourceAttribute, StatusCode

from . import tcpip
from .sessions import Session, VISARMSession

# dictionary lookup for Prologix controllers that have been opened
BOARDS = dict()


@Session.register(constants.InterfaceType.prlgx_tcpip, "INSTR")
class PrologixTCPIPInstrSession(tcpip.TCPIPSocketSession):

    def __init__(
        self,
        resource_manager_session: VISARMSession,
        resource_name: str,
        parsed: Optional[rname.ResourceName] = None,
        open_timeout: Optional[int] = None,
    ) -> None:
        super().__init__(resource_manager_session, resource_name, parsed, open_timeout)

        self.set_attribute(ResourceAttribute.termchar, ord("\n"))
        self.set_attribute(ResourceAttribute.termchar_enabled, True)

        # Set mode as CONTROLLER
        self.write(b"++mode 1\n")

        # Turn off read-after-write to avoid "Query Unterminated" errors
        self.write(b"++auto 0\n")

        # Read timeout is 500 msec
        # (code from Willow Garage, Inc uses 50ms)
        self.write(b"++read_tmo_ms 500\n")

        # Do not append CR or LF to GPIB data
        self.write(b"++eos 3\n")

        # Assert EOI with last byte to indicate end of data
        self.write(b"++eoi 1\n")

        if False:
            # additional setup found in code from Willow Garage, Inc
            self.write(b"++eot_enable 1\n")
            self.write(b"++eot_char 0\n")

        BOARDS[self.parsed.board] = self
        self._gpib_addr = ""

    def close(self) -> StatusCode:
        BOARDS.pop(self.parsed.board)
        return super().close()

    @property
    def gpib_addr(self) -> str:
        """
        gpib_addr is the currently addressed gpib instrument
        """
        return self._gpib_addr

    @gpib_addr.setter
    def gpib_addr(self, addr: str) -> None:
        if self._gpib_addr != addr:
            self.write(f"++addr {addr}\n".encode())
            self._gpib_addr = addr


class PrologixInstrSession(Session):
    # we don't decorate this class with Session.register() because we don't
    # want it to be registered in the _session_classes array, but we still
    # need to define session_type to make the set_attribute machinery work.
    session_type = (constants.InterfaceType.gpib, "INSTR")

    # Override parsed to take into account the fact that this class is only used
    # for a specific kind of resource
    parsed: rname.GPIBInstr

    @staticmethod
    def list_resources() -> List[str]:
        # TODO: is there a way to get this?
        return []

    def after_parsing(self) -> None:
        self.interface = BOARDS[self.parsed.board]
        self.gpib_addr = self.parsed.primary_address
        if self.parsed.secondary_address:
            # Secondary address of the device to connect to
            # Reference for the GPIB secondary address
            # https://www.mathworks.com/help/instrument/secondaryaddress.html
            # NOTE: a secondary address of 0 is not the same as no secondary address.
            self.gpib_addr += " " + self.parsed.secondary_address

    def close(self) -> StatusCode:
        if self.interface:
            self.interface = None
            return StatusCode.success

        return StatusCode.error_connection_lost

    def read(self, count: int) -> Tuple[bytes, StatusCode]:
        if self.interface:
            self.interface.gpib_addr = self.gpib_addr
            self.interface.write(b"++read eoi\n")
            return self.interface.read(count)

        return (b"", StatusCode.error_connection_lost)

    def write(self, data: bytes) -> Tuple[int, StatusCode]:
        if self.interface:
            self.interface.gpib_addr = self.gpib_addr
            # if the calling function has appended a newline to the data,
            # we don't want it to be escaped.  remove it from the data
            # and stash it away so we can append it after all the escapes
            # have been added in.
            if data[-2:] == b"\r\n":
                last_byte = b"\r\n"
                data = data[:-2]
            elif data[-1] == ord("\n"):
                last_byte = b"\n"
                data = data[:-1]
            else:
                last_byte = b""

            data = data.replace(b"\033", b"\033\033")
            data = data.replace(b"\n", b"\033\n")
            data = data.replace(b"\r", b"\033\r")
            data = data.replace(b"+", b"\033+")
            return self.interface.write(data + last_byte)

        return (0, StatusCode.error_connection_lost)

    def flush(self, mask: BufferOperation) -> StatusCode:
        if self.interface:
            return self.interface.flush(mask)

        return StatusCode.error_connection_lost

    def clear(self) -> StatusCode:
        """Clears a device.

        Corresponds to viClear function of the VISA library.

        Returns
        -------
        StatusCode
            Return value of the library call.

        """
        logger.debug("GPIB.device clear")
        if self.interface:
            self.interface.gpib_addr = self.gpib_addr
            _, status_code = self.interface.write(b"++clr\n")
            return status_code

        return StatusCode.error_connection_lost

    def assert_trigger(self, protocol: constants.TriggerProtocol) -> StatusCode:
        """Asserts hardware trigger.

        Parameters
        ----------
        protocol : constants.TriggerProtocol
            Triggering protocol to use.
            Only supports constants.TriggerProtocol.default

        Returns
        -------
        StatusCode
            Return value of the library call.

        """
        logger.debug("GPIB.device assert hardware trigger")

        if self.interface:
            self.interface.gpib_addr = self.gpib_addr
            _, status_code = self.interface.write(b"++trg\n")
            return status_code

        return StatusCode.error_connection_lost

    def read_stb(self) -> Tuple[int, StatusCode]:
        """Read the device status byte."""
        if self.interface:
            self.interface.gpib_addr = self.gpib_addr
            self.interface.write(b"++spoll\n")
            data, status_code = self.interface.read(32)
            return (int(data), status_code)

        return (-1, StatusCode.error_connection_lost)

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
