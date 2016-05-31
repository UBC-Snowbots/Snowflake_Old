import enum as _enum
import bitstring as _bitstring

import jaus.format as _format

_specs = {}

def define_message(name, specs, defaults={}):
    code = getattr(MessageCode, name)
    specification = _format.specification(name, specs, defaults)
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

    ## Management

    Shutdown = 0x0002
    Standby = 0x0003
    Resume = 0x0004
    Reset = 0x0005
    SetEmergency = 0x0006
    ClearEmergency = 0x0007
    QueryStatus = 0x2002

    ReportStatus = 0x4002

    ## List Manager Service
    #Input Set
    SetElement = 0x041A
    DeleteElement = 0x041B
    QueryElement = 0x241A
    QueryElementList = 0x241B
    QueryElementCount = 0x241C
    #Output Set
    ConfirmElementRequest = 0x041C
    RejectElementRequest = 0x041D
    ReportElement = 0x441A
    ReportElementList = 0x441B
    ReportElementCount = 0x441C

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
], defaults={
    'message_code': None,
    'event_type': None,
    'event_id': None,
    'all_events': lambda attrs: 0 if attrs['variant'] is QueryEventsVariant.ALL_EVENTS else None,
})
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
], defaults={
    'presence_vector': lambda attrs: 1 if attrs['error_message'] is not None else 0,
    'error_message': None,
})

ReportEvent = _format.specification('ReportEvent', [
    _format.Enum('event_type', enum=EventType, bytes=1),
    _format.Int('event_id', bytes=1),
    CountedBytes('query_message', bytes=4, endianness='le'),
])
define_message('ReportEvents', [
    _format.Int('count', bytes=1),
    _format.Repeat('events', ReportEvent, count=lambda attrs: attrs['count']),
], defaults={'count': lambda attrs: len(attrs['events'])})
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
    NOT_AVAILABLE = 1
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

### Management

define_message('Shutdown', [])
define_message('Standby', [])
define_message('Resume', [])
define_message('Reset', [])
class EmergencyCode(_enum.Enum):
    STOP = 1
define_message('SetEmergency', [
    _format.Enum('emergency_code', enum=EmergencyCode, bytes=1),
])
define_message('ClearEmergency', [
    _format.Enum('emergency_code', enum=EmergencyCode, bytes=1),
])
define_message('QueryStatus', [])

class ManagementStatus(_enum.Enum):
    INIT = 0
    READY = 1
    STANDBY = 2
    SHUTDOWN = 3
    FAILURE = 4
    EMERGENCY = 5
define_message('ReportStatus', [
    _format.Enum('status', enum=ManagementStatus, bytes=1),
    _format.Int('reserved', bytes=4, endianness='le'),
], defaults={'reserved': 0})

### ListManager

ListElement = _format.specification('ListElement', [
    _format.Int('uid', bytes=2, endianness='le'),
    _format.Int('prev', bytes=2, endianness='le'),
    _format.Int('next', bytes=2, endianness='le'),
    CountedBytes('data', bytes=2, endianness='le'),
])
define_message('SetElement', [
    _format.Int('request_id', bytes=1),
    _format.Int('element_count', bytes=1),
    _format.Repeat('elements', ListElement, count=lambda attrs: attrs['element_count']),
], defaults={'element_count': lambda attrs: len(attrs['elements'])})
ListElementID = _format.specification('ListElementID', [
    _format.Int('uid', bytes=2, endianness='le'),
])
define_message('DeleteElement', [
    _format.Int('request_id', bytes=1),
    _format.Int('element_count', bytes=1),
    _format.Repeat('element_ids', ListElementID, count=lambda attrs: attrs['element_count']),
], defaults={'element_count': lambda attrs: len(attrs['element_ids'])})
define_message('QueryElement', [
    _format.Int('element_uid', bytes=2, endianness='le'),
])
define_message('QueryElementList', [])
define_message('QueryElementCount', [])

define_message('ConfirmElementRequest', [
    _format.Int('request_id', bytes=1),
])
class RejectElementRequestResponseCode(_enum.Enum):
    INVALID_ELEMENT_ID = 1
    INVALID_PREVIOUS_ELEMENT = 2
    INVALID_NEXT_ELEMENT = 3
    UNSUPPORTED_ELEMENT_TYPE = 4
    ELEMENT_ID_NOT_FOUND = 5
    OUT_OF_MEMORY = 6
    UNSPECIFIED_ERROR = 7
define_message('RejectElementRequest', [
    _format.Int('request_id', bytes=1),
    _format.Enum('response_code', enum=RejectElementRequestResponseCode, bytes=1),
])
define_message('ReportElement', [
    _format.Instance('element', ListElement),
])
define_message('ReportElementList', [
    _format.Int('element_count', bytes=2, endianness='le'),
    _format.Repeat('elements', ListElementID, count=lambda attrs: attrs['element_count']),
], defaults={'element_count': lambda attrs: len(attrs['elements'])})
define_message('ReportElementCount', [
    _format.Int('element_count', bytes=2, endianness='le'),
])
