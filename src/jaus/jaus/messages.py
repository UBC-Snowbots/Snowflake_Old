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

class CountedBytes(_format.NamedSpec):
    def __init__(self, name, *args, **kwargs):
        super().__init__(name)
        self.group = _format.Group([
            _format.Int('count', *args, **kwargs),
            _format.Bytes('contents', length=lambda attrs: attrs['count']),
        ])
    def read_one(self, buf, attributes):
        result = self.group.read(buf, {})
        return result['contents']
    def write_one(self, buf, prop, _):
        self.group.write(buf, {
            'count': len(prop),
            'contents': prop,
        })

class ScaledFloat(_format.Int):
    def __init__(self, name, real_lower_limit, real_upper_limit, *args, **kwargs):
        super().__init__(name, *args, **kwargs)
        self.real_lower_limit = real_lower_limit
        self.real_upper_limit = real_upper_limit
    def read_one(self, *args, **kwargs):
        result = super().read_one(*args, **kwargs)
        return (result/self.max * self.real_range) + self.real_lower_limit
    @property
    def real_range(self):
        return self.real_upper_limit - self.real_lower_limit
    def write_one(self, buf, prop, attributes):
        encoded_value = round(
            (prop - self.real_lower_limit)/self.real_range*self.max)
        super().write_one(buf, encoded_value, attributes)

class MessageCode(_enum.Enum):
    QueryHeartbeatPulse = 0x2202
    ReportHeartbeatPulse = 0x4202

    ## Events
    CreateEvent = 0x01F0
    UpdateEvent = 0x01F1
    CancelEvent = 0x01F2
    CreateCommandEvent = 0x01F6
    QueryEvents = 0x21F0
    QueryEventTimeout = 0x21F2

    ConfirmEventRequest = 0x01F3
    RejectEventRequest = 0x01F4
    ReportEvents = 0x41F0
    Event = 0x41F1
    ReportEventTimeout = 0x41F2
    CommandEvent = 0x41F6

    ## Access Control
    RequestControl = 0x000D
    ReleaseControl = 0x000E
    QueryControl = 0x200D
    QueryAuthority = 0x2001
    SetAuthority = 0x0001
    QueryTimeout = 0x2003

    ReportControl = 0x400D
    RejectControl = 0x0010
    ConfirmControl = 0x000F
    ReportAuthority = 0x4001
    ReportTimeout = 0x4003

Message = _format.specification('Message', specs=[
        _format.Enum('message_code', MessageCode, bytes=2, endianness='le'),
        _format.Switch('body', lambda attrs: _specs[attrs['message_code']]),
    ])

Id = _format.specification('Id', [
    _format.Int('subsystem', bytes=2, endianness='le'),
    _format.Int('node', bytes=1),
    _format.Int('component', bytes=1),
])

### Liveness

define_message('QueryHeartbeatPulse', [])
define_message('ReportHeartbeatPulse', [])

### Events

class EventType(_enum.Enum):
    PERIODIC = 0
    EVERY_CHANGE = 1

define_message('CreateEvent', [
    _format.Int('request_id', bytes=1),
    _format.Enum('event_type', enum=EventType, bytes=1),
    ScaledFloat(
        'requested_periodic_rate',
        bytes=2,
        endianness='le',
        real_lower_limit=0,
        real_upper_limit=1092),
    CountedBytes('query_message', bytes=4, endianness='le'),
])
define_message('UpdateEvent', [
    _format.Int('request_id', bytes=1),
    _format.Enum('event_type', enum=EventType, bytes=1),
    ScaledFloat(
        'requested_periodic_rate',
        bytes=2,
        endianness='le',
        real_lower_limit=0,
        real_upper_limit=1092),
    _format.Int('event_id', bytes=1),
    CountedBytes('query_message', bytes=4, endianness='le'),
])
define_message('CancelEvent', [
    _format.Int('request_id', bytes=1),
    _format.Int('event_id', bytes=1),
])
define_message('CreateCommandEvent', [
    _format.Int('request_id', bytes=1),
    _format.Int('maximum_allowed_duration', bytes=4, endianness='le'),
    CountedBytes('command_message', bytes=4, endianness='le'),
])

class QueryEventsVariant(_enum.Enum):
    MESSAGE_ID = 0
    EVENT_TYPE = 1
    EVENT_ID = 2
    ALL_EVENTS = 3

define_message('QueryEvents', [
    _format.Enum('variant', enum=QueryEventsVariant, bytes=1),
    _format.Optional(lambda attrs: attrs['variant'] is QueryEventsVariant.MESSAGE_ID, [
        _format.Enum('message_code', enum=MessageCode, bytes=2, endianness='le'),
    ]),
    _format.Optional(lambda attrs: attrs['variant'] is QueryEventsVariant.EVENT_TYPE, [
        _format.Enum('event_type', enum=EventType, bytes=1),
    ]),
    _format.Optional(lambda attrs: attrs['variant'] is QueryEventsVariant.EVENT_ID, [
        _format.Int('event_id', bytes=1),
    ]),
    _format.Optional(lambda attrs: attrs['variant'] is QueryEventsVariant.ALL_EVENTS, [
        _format.Int('all_events', bytes=1),
    ]),
])
define_message('QueryEventTimeout', [])
define_message('ConfirmEventRequest', [
    _format.Int('request_id', bytes=1),
    _format.Int('event_id', bytes=1),
    ScaledFloat(
        'confirmed_periodic_rate',
        bytes=2,
        endianness='le',
        real_lower_limit=0,
        real_upper_limit=1092),
])

class RejectEventRequestResponseCode(_enum.Enum):
    PERIODIC_EVENTS_NOT_SUPPORTED = 1
    CHANGE_BASED_EVENTS_NOT_SUPPORTED = 2
    CONNECTION_REFUSED = 3
    INVALID_EVENT_SETUP = 4
    MESSAGE_NOT_SUPPORTED = 5
    INVALID_EVENT_ID_FOR_UPDATE = 6

define_message('RejectEventRequest', [
    _format.Int('presence_vector', bytes=1),
    _format.Int('request_id', bytes=1),
    _format.Enum('response_code', enum=RejectEventRequestResponseCode, bytes=1),
    _format.Optional(lambda attrs: attrs['presence_vector'] != 0, [
        _format.Bytes('error_message', length=80),
    ])
])

ReportEvent = _format.specification('ReportEvent', [
    _format.Enum('event_type', enum=EventType, bytes=1),
    _format.Int('event_id', bytes=1),
    CountedBytes('query_message', bytes=4, endianness='le'),
])
define_message('ReportEvents', [
    _format.Int('count', bytes=1),
    _format.Repeat('events', ReportEvent, count=lambda attrs: attrs['count']),
])
define_message('Event', [
    _format.Int('event_id', bytes=1),
    _format.Int('sequence_number', bytes=1),
    CountedBytes('report_message', bytes=4, endianness='le'),
])
define_message('ReportEventTimeout', [
    _format.Int('timeout', bytes=1),
])

class CommandResult(_enum.Enum):
    SUCCESSFUL = 0
    UNSUCCESSFUL = 1
define_message('CommandEvent', [
    _format.Int('event_id', bytes=1),
    _format.Enum('command_result', enum=CommandResult, bytes=1),
])

### Access Control

define_message('RequestControl', [
    _format.Int('authority_code', bytes=1),
])
define_message('ReleaseControl', [])
define_message('QueryControl', [])
define_message('QueryAuthority', [])
define_message('SetAuthority', [
    _format.Int('authority_code', bytes=1),
])
define_message('QueryTimeout', [])

define_message('ReportControl', [
    _format.Instance('id', Id),
    _format.Int('authority_code', bytes=1),
])
class RejectControlResponseCode(_enum.Enum):
    CONTROL_RELEASED = 0
    CONTROL_NOT_AVAILABLE = 1
define_message('RejectControl', [
    _format.Enum('response_code', enum=RejectControlResponseCode, bytes=1),
])
class ConfirmControlResponseCode(_enum.Enum):
    CONTROL_ACCEPTED = 0
    NOT_AVAILABLE = 1
    INSUFFICIENT_AUTHORITY = 2
define_message('ConfirmControl', [
    _format.Enum('response_code', enum=ConfirmControlResponseCode, bytes=1),
])
define_message('ReportAuthority', [
    _format.Int('authority_code', bytes=1),
])
define_message('ReportTimeout', [
    _format.Int('timeout', bytes=1),
])
