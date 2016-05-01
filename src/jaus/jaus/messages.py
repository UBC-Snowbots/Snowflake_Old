import enum as _enum
import bitstring as _bitstring

import jaus.format as _format

_specs = {}

def define_message(name, specs):
    code = getattr(MessageCode, name)
    specification = _format.specification(name, specs)
    globals()[name] = specification
    _specs[code] = specification

    message_type_name = '{}Message'.format(name)
    def instantiate_message_shortcut(*args, **kwargs):
        return Message(
          message_code=code,
          body=specification(*args, **kwargs))
    globals()[message_type_name] = instantiate_message_shortcut

def assemble_message(packets):
    cumulative = _bitstring.BitStream()
    for packet in packets:
        cumulative.append(_bitstring.BitString(bytes=packet.contents))
        return Message._instantiate(cumulative)


class MessageCode(_enum.Enum):
    QueryHeartbeatPulse = 0x2202
    ReportHeartbeatPulse = 0x4202

Message = _format.specification('Message', specs=[
        _format.Enum('message_code', MessageCode, bytes=2, endianness='le'),
        _format.Switch('body', lambda attrs: _specs[attrs['message_code']]),
    ])

define_message('QueryHeartbeatPulse', [])
define_message('ReportHeartbeatPulse', [])
