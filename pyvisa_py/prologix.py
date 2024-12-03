from typing import Optional

from pyvisa import constants, rname
from pyvisa.constants import ResourceAttribute

from . import tcpip
from .sessions import Session, VISARMSession


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
        self.write(b"++read_tmo_ms 500\n")

        # Do not append CR or LF to GPIB data
        self.write(b"++eos 3\n")

        # Assert EOI with last byte to indicate end of data
        self.write(b"++eoi 1\n")
