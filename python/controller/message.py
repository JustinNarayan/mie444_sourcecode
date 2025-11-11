# message.py
from enum import Enum, auto
import struct

MESSAGE_END_CHAR = b"$"
NULL_TERMINATOR = b"\x00"
ENCODING_MINIMUM_LENGTH = 3  # type char, size char, end char


# Type of Message.
#
# Corresponds to lib/Message/MessageType.h
class MessageType(Enum):
    def _generate_next_value_(name, start, count, last_values):
        return count

    Unused = auto()
    Generic = auto()
    Error = auto()

    DrivetrainManualCommand = auto()
    DrivetrainManualResponse = auto()
    DrivetrainEncoderRequest = auto()
    DrivetrainEncoderPinged = auto()
    DrivetrainEncoderDistances = auto()

    LidarRequest = auto()
    LidarPointReading = auto()
    LidarComplete = auto()

    Count = auto()


# Registry for the format of all received content.
#
# Similar to Translate.h
_TYPE_FORMATS = {
    MessageType.DrivetrainEncoderDistances: dict(
        fmt="<fff",  # three float32_t
        units=("in",) * 3,  # all in cm
        disp="{:.2f} {u}",  # display format
    ),
    MessageType.LidarPointReading: dict(
        fmt="<hh",  # two int16_t
        units=("°", "mm"),  # degree, mm
        disp="{}{u}",  # display format
    ),
    MessageType.Generic: dict(text=True),
    MessageType.Error: dict(text=True),
}

SHOULD_NOT_PRINT_TO_SCREEN = [
	# MessageType.DrivetrainEncoderDistances,
	# MessageType.DrivetrainManualCommand,
	# MessageType.LidarPointReading,
]

# Message class
#
# Corresponds to lib/Message/Message.h
class Message:

    def __init__(self, type: MessageType, content: bytes):
        """
        Construct a Message directly from type and content bytes.
        """
        self.type = type
        self.content = content
        self.size = len(content)
        self.raw = self.encode()
        
    def get_type(self):
        return self.type

    @classmethod
    def from_raw(self, raw: bytes):
        """
        Construct a Message object from a raw byte string.
        Expected format: [type][size][content...]['$'][optional '\0']
        """
        if len(raw) < ENCODING_MINIMUM_LENGTH:
            raise ValueError("Raw message too short")

        # Extract first two bytes
        type_val = raw[0]
        size = raw[1]

        # Decode type
        try:
            msg_type = MessageType(type_val)
        except ValueError:
            raise ValueError(f"Invalid message type value: {type_val}")

        # Extract content and validate terminator
        content = raw[2 : 2 + size]

        # Validate the end char
        expected_end = raw[2 + size : 3 + size]
        if expected_end != MESSAGE_END_CHAR:
            raise ValueError(f"Missing or invalid message end char: {expected_end}")

        # Remove null terminator if present
        raw = raw.rstrip(NULL_TERMINATOR)

        # Construct message
        msg = self(msg_type, content)
        msg.raw = raw
        return msg

    def encode(self) -> bytes:
        """
        Return the full encoded message as bytes.
        Format: [type][size][content...]['$'][optional '\0']
        """
        return (
            self.type.value.to_bytes(1, "little")
            + self.size.to_bytes(1, "little")
            + self.content
            + MESSAGE_END_CHAR
        )

    def _decode_struct(self, meta):
        """
        Helper to unpack struct into the format expected based on the MessageType
        """
        fmt = meta["fmt"]
        expected = struct.calcsize(fmt)
        if len(self.content) != expected:
            raise ValueError(
                f"{self.type.name}: expected {expected} bytes, got {len(self.content)}"
            )
        values = struct.unpack(fmt, self.content)
        return values

    def decode(self):
        """
        Decode a Message into the correct information format based on the metadata of the MessageType
        """
        meta = _TYPE_FORMATS.get(self.type)
        if not meta:
            # no known structure
            return self.content.decode(errors="replace") if self.content else None
        if meta.get("text"):
            return self.content.decode(errors="replace")
        if "fmt" in meta:
            return self._decode_struct(meta)
        return self.content

    def __repr__(self):
        """
		Get string representation of Message
        """
        meta = _TYPE_FORMATS.get(self.type)
        val = self.decode()
        if meta and "fmt" in meta:
            # metadata will include formatting method
            disp, units = meta["disp"], meta["units"]
            parts = [
                disp.format(v, u=units[i] if i < len(units) else "")
                for i, v in enumerate(val)
            ]
            return f"<{self.type.name}({', '.join(parts)})>"
        
        if meta and meta.get("text"):
			# display raw text representation
            return f"<{self.type.name}({val})>"
        
        if not self.content:
            # no content to display
            return f"<{self.type.name}()>"
        
        # raw bytes representation as last resort
        try:
            return f"<{self.type.name}({self.content.decode(errors='replace')})>"
        except Exception:
            return f"<{self.type.name}({self.content!r})>"

# Helps parse received bytes on Serial into Messages
#
def parse_message_from_buffer(buffer: bytes):
    """
    Try to parse a Message from the start of buffer.
    Returns: (msg, bytes_consumed)
    Raises ValueError if the message is invalid.
    """
    if len(buffer) < 2:
        return None, 0  # not enough bytes for header

    type_byte = buffer[0]
    size = buffer[1]
    total_len = 2 + size + 1  # type + size + content + '$'

    if len(buffer) < total_len:
        return None, 0  # not enough bytes yet

    raw_msg = buffer[:total_len]
    msg = Message.from_raw(raw_msg)
    return msg, total_len