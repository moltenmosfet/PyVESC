import sys
if sys.version_info < (3, 6):
    raise SystemExit("Invalid Python version. PyVESC requires Python 3.6 or greater.")

from .protocol.interface import encode, encode_request, decode
from .messages import *
from .VESC import VESC
from .transport import TCPTransport
