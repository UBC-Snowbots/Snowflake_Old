import asyncio as _asyncio
import functools as _functools
import bitstring as _bitstring

import jaus.messages as _messages
import jaus.judp as _judp
import jaus.signal as _signal


def message_handler(message_code, is_command=False, supports_events=True):
    def process(fn):
        fn.is_message_handler = True
        fn.message_code = message_code
        fn.is_command = is_command
        fn.supports_events = supports_events
        return fn
    return process

class ServiceMeta(type):
    def __init__(klass, name, bases, dct):
        message_handlers = {}
        for name, val in dct.items():
            if hasattr(val, 'is_message_handler'):
                assert val.message_code not in message_handlers
                # store the names, in case someone ever needs them
                message_handlers[val.message_code] = name
        klass.message_handlers = message_handlers
        return super().__init__(name, bases, dct)

class Service(metaclass=ServiceMeta):
    def __init__(self, component):
        super().__init__()
        self.component = component
        for message_code in self.message_handlers:
            message_handler = self._get_message_handler(message_code)
            component.transport.message_received.connect(
                message_handler,
                message_code=message_code)
            if message_handler.supports_events:
                component.events.register_event(message_code, message_handler)
    def _get_message_handler(self, message_code):
        return getattr(self, self.message_handlers[message_code])

class Component(object):
    def __init__(self, id, services = []):
        self.id = id
        self.services = {}

        for service in services:
            instance = service(component=self)
            self.services[instance.name] = instance

    def __getattr__(self, name):
        if name in self.services:
            return self.services[name]
        else:
            raise AttributeError(name)

class Transport(Service):
    name='transport'

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.message_received = _signal.Signal(key='message_code')
        self.protocol = _judp.JUDPProtocol(
            message_received=self.on_message_received,
            own_id=self.component.id)

    @_asyncio.coroutine
    def on_message_received(self, source_id, message):
        responses = yield from _asyncio.gather(*self.message_received.send(
            message_code=message.message_code,
            message=message,
            source_id=source_id))
        yield from _asyncio.gather(*[
            self.send_message(message=response, destination_id=source_id)
            for response in responses
        ])

    @_asyncio.coroutine
    def send_message(self, *args, **kwargs):
        yield from self.protocol.send_message(*args, **kwargs)

class Event:
    def __init__(self, timeout, process, id, destination_id, message, type, periodic_rate, request_id):
        self.timeout = timeout
        self.id = id
        self.process = process
        self.sequence_number = 0
        self.destination_id = destination_id
        self.message = message
        self.type = type
        self.periodic_rate = periodic_rate
        self.request_id = request_id
    def stop(self):
        self.timeout.cancel()
        self.process.cancel()

class Events(Service):
    name='events'

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.handlers = {}
        self.events = {}
        self._next_event_id = 0
        self.event_timeout = 60

    def register_event(self, message_code, handler):
        self.handlers[message_code] = handler

    @_asyncio.coroutine
    def _fire_event(self, event):
        message_code = event.message.message_code
        handler = self.handlers[message_code]
        response = yield from handler(
            message=event.message,
            message_code=message_code)

        yield from self.component.transport.send_message(
            destination_id=event.destination_id,
            message=_messages.EventMessage(
                event_id=event.id,
                sequence_number=event.sequence_number,
                report_message=response._serialize_to_bytes()))
        event.sequence_number += 1
        if event.sequence_number > 255:
            event.sequence_number = 0
    @_asyncio.coroutine
    def _event_timeout(self, event_id):
        event = self.events[event_id]
        print("Sleeping for {}".format(self.event_timeout))
        yield from _asyncio.sleep(self.event_timeout)
        print("Wake up, stop event")
        event.process.cancel()
        del self.events[event.id]
        yield from self.component.transport.send_message(
            destination_id=event.destination_id,
            message=_messages.ConfirmEventRequestMessage(
                request_id=event.request_id,
                event_id=event.id,
                confirmed_periodic_rate=event.periodic_rate))
        print("Send confirmation")
    @_asyncio.coroutine
    def _process_event(self, event_id):
        event = self.events[event_id]
        if event.type is _messages.EventType.PERIODIC:
            while True:
                yield from self._fire_event(event)
                yield from _asyncio.sleep(1/event.periodic_rate)

    def _get_event_id(self):
        event_id = self._next_event_id
        self._next_event_id += 1
        return event_id

    def _normalise_periodic_rate(self, requested_periodic_rate, event_type):
        if event_type is _messages.EventType.EVERY_CHANGE:
            return 0
        else:
            return 5

    @_asyncio.coroutine
    def post_change(self, message_codes):
        for event in self.events.values:
            if (event.message.message_code in message_codes
                    and event.type is _messages.EventType.EVERY_CHANGE):
                yield from self._fire_event(event)

    @message_handler(
        _messages.MessageCode.CreateEvent,
        is_command=True,
        supports_events=False)
    @_asyncio.coroutine
    def on_create_event(self, source_id, message, **kwargs):
        message = message.body
        event_id = self._get_event_id()
        periodic_rate = self._normalise_periodic_rate(
            message.requested_periodic_rate,
            message.event_type)
        event = Event(
            id=event_id,
            destination_id=source_id,
            message=_messages.Message._instantiate(_bitstring.ConstBitStream(
                message.query_message)),
            type=message.event_type,
            timeout=_asyncio.async(self._event_timeout(event_id)),
            process=_asyncio.async(self._process_event(event_id)),
            periodic_rate=periodic_rate,
            request_id=message.request_id)
        self.events[event_id] = event
        return _messages.ConfirmEventRequestMessage(
            request_id=message.request_id,
            event_id=event_id,
            confirmed_periodic_rate=periodic_rate)

    @message_handler(
        _messages.MessageCode.UpdateEvent,
        is_command=True,
        supports_events=False)
    @_asyncio.coroutine
    def on_update_event(self, source_id, message, **kwargs):
        message = message.body
        if message.event_id not in self.events:
            return _messages.RejectEventRequestMessage(
                presence_vector=0,
                request_id=message.request_id,
                response_code=_messages.RejectEventRequestResponseCode.INVALID_EVENT_ID_FOR_UPDATE,
                error_message=None)
        periodic_rate = self._normalise_periodic_rate(
            message.requested_periodic_rate,
            message.event_type)
        event = Event(
            id=message.event_id,
            destination_id=source_id,
            message=_messages.Message._instantiate(message.query_message),
            type=message.event_type,
            timeout=_asyncio.async(timeout),
            process=_asyncio.async(process),
            periodic_rate=periodic_rate,
            request_id=message.request_id)
        self.events[message.event_id].stop()
        self.events[message.event_id] = event
        return _messages.ConfirmEventRequestMessage(
            request_id=message.request_id,
            event_id=message.event_id,
            confirmed_periodic_rate=periodic_rate)

    @message_handler(
        _messages.MessageCode.CancelEvent,
        is_command=True,
        supports_events=False)
    @_asyncio.coroutine
    def on_cancel_event(self, message, **kwargs):
        message = message.body
        if message.event_id in self.events:
            event = self.events[message.event_id]
            event.stop()
            return _messages.ConfirmEventRequestMessage(
                request_id=message.request_id,
                event_id=message.event_id,
                confirmed_periodic_rate=event.periodic_rate)
        else:
            return _messages.RejectEventRequestMessage(
                presence_vector=0,
                request_id=message.request_id,
                response_code=_messages.RejectEventRequestResponseCode.INVALID_EVENT_ID_FOR_UPDATE,
                error_message=None)

    @message_handler(
        _messages.MessageCode.QueryEvents,
        supports_events=False)
    @_asyncio.coroutine
    def on_query_events(self, message, **kwargs):
        message = message.body
        variant = message.variant
        def report_event(event):
            return _messages.ReportEvent(
                event_type=event.type,
                event_id=event.id,
                query_message=event.message._serialize_to_bytes())
        if variant is _messages.QueryEventsVariant.MESSAGE_ID:
            predicate = lambda event: event.message.message_code is message.message_code
        elif variant is _messages.QueryEventsVariant.EVENT_TYPE:
            predicate = lambda event: event.type is message.event_type
        elif variant is _messages.QueryEventsVariant.EVENT_ID:
            predicate = lambda event: event.id == message.event_id
        elif variant is _messages.QueryEventsVariant.ALL_EVENTS:
            predicate = lambda event: True

        report = [
                report_event(event)
                for event in self.events.values()
                if predicate(event)]
        return _messages.ReportEventsMessage(
            count=len(report),
            events=report)

    @message_handler(
        _messages.MessageCode.QueryEventTimeout,
        supports_events=False)
    @_asyncio.coroutine
    def on_query_event_timeout(self, **kwargs):
        return _messages.ReportEventTimeoutMessage(
            timeout=int(self.event_timeout/60))

class Liveness(Service):
    name='liveness'

    @message_handler(_messages.MessageCode.QueryHeartbeatPulse)
    @_asyncio.coroutine
    def on_query_heartbeat(self, **kwargs):
        return _messages.ReportHeartbeatPulseMessage()

class Discovery(Service):
    name='discovery'
    @message_handler(
        _messages.MessageCode.QueryIdentification,
        is_command=True,
        supports_events=False)
    @_asyncio.coroutine
    def RegisterServices(self, message, URI, majorVersionNumber, minorVersionNumber): # message that allows component to register its capabilities with a discovery service. 
        message = message.body
        if(id == 1):
            # Node manager, can provide any number of services in addition to core msg supp.


    @message_handler(_messages.MessageCode.QueryIdentification)
    @_asyncio.coroutine
    def on_query_identification():
        message = message.body

    @message_handler(_messages.MessageCode.QueryConfiguration)
    @_asyncio.coroutine
    def on_query_configuration():

    @message_handler(_messages.MessageCode.QuerySubsystemList)
    @_asyncio.coroutine
    def on_query_subsystem_list():

    @message_handler(_messages.MessageCode.QueryServices)
    @_asyncio.coroutine
    def on_query_services():

    @message_handler(_messages.MessageCode.QueryServiceList)
    @_asyncio.coroutine
    def on_query_service_list():
    
    
    # basically there are two things to take consideration of,
    # input data via register actions
    # return answers from queries, even
    # Store all info of register actions into a table or map.
    # Handlers of event class contains info to get info, have dict, set get vals.