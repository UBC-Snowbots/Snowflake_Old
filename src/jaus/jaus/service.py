import asyncio as _asyncio
import functools as _functools
import bitstring as _bitstring
import logging as _logging

import jaus.messages as _messages
import jaus.judp as _judp
import jaus.signal as _signal
import jaus.list_manager as _list_manager


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
            # allow access_control to manipulate messages before we get them
            # but only if they're commands
            # (do not use if your service doesn't inherit from AccessControl)
            if message_handler.is_command:
                proxy = component.access_control.make_command_proxy(
                    message_handler)
            else:
                proxy = message_handler
            component.transport.message_received.connect(
                proxy,
                message_code=message_code)
            # allow events to call the handler when it needs
            if message_handler.supports_events:
                component.events.register_event(message_code, message_handler)
    def _get_message_handler(self, message_code):
        return getattr(self, self.message_handlers[message_code])
    def close(self):
        pass

class Component(object):
    def __init__(self, id, services = [], default_authority=0):
        self.id = id
        self.services = {}
        self.default_authority = default_authority

        for service in services:
            instance = service(component=self)
            self.services[instance.name] = instance

    def __getattr__(self, name):
        if name in self.services:
            return self.services[name]
        else:
            raise AttributeError(name)

    def close(self):
        for s in self.services.values():
            if not isinstance(s, Transport):
                s.close()
            else:
                transport = s
        transport.close()

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
            # check for none as sometimes no response should be sent
            if response is not None
        ])

    @_asyncio.coroutine
    def send_message(self, *args, **kwargs):
        yield from self.protocol.send_message(*args, **kwargs)

    def close(self):
        super().close()
        self.protocol.close()

def change_watcher(backing, query_codes):
    @property
    def prop(self):
        return getattr(self, backing)
    @prop.setter
    def prop(self, val):
        if val != getattr(self, backing):
            _asyncio.async(
                self.component.events.post_change(message_codes=query_codes))
        setattr(self, backing, val)
    return prop

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

    def close(self):
        super().close()
        for e in self.events.values():
            e.stop()

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
        yield from _asyncio.sleep(self.event_timeout)
        event.process.cancel()
        del self.events[event.id]
        yield from self.component.transport.send_message(
            destination_id=event.destination_id,
            message=_messages.ConfirmEventRequestMessage(
                request_id=event.request_id,
                event_id=event.id,
                confirmed_periodic_rate=event.periodic_rate))
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
        for event in self.events.values():
            if (event.message.message_code in message_codes
                    and event.type is _messages.EventType.EVERY_CHANGE):
                yield from self._fire_event(event)

    @message_handler(
        _messages.MessageCode.CreateEvent,
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

class AccessControl(Service):
    name='access_control'

    # Watchable attributes
    controlling_component = change_watcher(
        '_controlling_component',
        query_codes=(_messages.MessageCode.QueryControl,))
    authority = change_watcher(
        '_authority',
        query_codes=(_messages.MessageCode.QueryAuthority,))

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._controlling_component = None
        self._authority = self.component.default_authority
        self.timeout_routine = _asyncio.async(self._timeout_routine())
        self.timeout = 5 # seconds

    def close(self):
        super().close()
        self.timeout_routine.cancel()

    @property
    def is_controlled(self):
        return self.controlling_component is not None

    def has_control(self, component_id):
        return self.controlling_component == component_id

    @_asyncio.coroutine
    def _timeout_routine(self):
        yield from _asyncio.sleep(self.timeout)
        if self.is_controlled:
            if not self.control_available:
                self.reset_timeout()
                return
            controlling_component = self.controlling_component
            _logging.info('Control by component {} timed out'
                .format(controlling_component))
            self.controlling_component = None
            yield from self.component.transport.send_message(
                destination_id=controlling_component,
                message=_messages.RejectControlMessage(
                    response_code=_messages.RejectControlResponseCode.CONTROL_RELEASED))

    def reset_timeout(self):
        self.timeout_routine.cancel()
        self.timeout_routine = _asyncio.async(self._timeout_routine())

    def make_command_proxy(self, handler):
        """Helper to make trivial has-authority-or-ignore-it commands easy to specify."""
        @_asyncio.coroutine
        @_functools.wraps(handler)
        def proxy(source_id, **kwargs):
            if self.has_control(source_id):
                return (yield from handler(source_id=source_id, **kwargs))
            else:
                # ignore commands with insufficient authority
                return None
        return proxy

    @message_handler(
        _messages.MessageCode.RequestControl,
        supports_events=False)
    @_asyncio.coroutine
    def on_request_control(self, source_id, message, **kwargs):
        message = message.body
        if not self.is_controlled:
            if self.control_available:
                if self.component.default_authority > message.authority_code:
                    return _messages.ConfirmControlMessage(
                        response_code=_messages.ConfirmControlResponseCode.INSUFFICIENT_AUTHORITY)
                else:
                    self.controlling_component = source_id
                    self.authority = message.authority_code
                    self.reset_timeout()
                    return _messages.ConfirmControlMessage(
                        response_code=_messages.ConfirmControlResponseCode.CONTROL_ACCEPTED)
            else:
                return _messages.ConfirmControlMessage(
                    response_code=_messages.ConfirmControlResponseCode.NOT_AVAILABLE)
        else:
            if self.control_available:
                if self.controlling_component == source_id:
                    if self.component.default_authority > message.authority_code:
                        # somehow the controller's authority decreased, so we reject them
                        self.reset_timeout()
                        self.controlling_component = None
                        return _messages.RejectControlMessage(
                            response_code=_messages.RejectControlResponseCode.CONTROL_RELEASED)
                    else:
                        # reset control info, same controller
                        self.authority = message.authority_code
                        self.reset_timeout()
                        return _messages.ConfirmControlMessage(
                            response_code=_messages.ConfirmControlMessage.CONTROL_ACCEPTED)
                else:
                    if self.authority < message.authority_code:
                        # new controller has greater authority than current; switch
                        self.authority = message.authority_code
                        yield from self.reject_control(source_id)
                        return _messages.ConfirmControlMessage(
                            response_code=_messages.ConfirmControlResponseCode.CONTROL_ACCEPTED)
                    else:
                        return _messages.ConfirmControlMessage(
                            response_code=_messages.ConfirmControlResponseCode.INSUFFICIENT_AUTHORITY)
            else:
                return _messages.ConfirmControlMessage(
                    response_code=_messages.ConfirmControlResponseCode.NOT_AVAILABLE)

    @message_handler(
        _messages.MessageCode.ReleaseControl,
        supports_events=False)
    @_asyncio.coroutine
    def on_release_control(self, source_id, **kwargs):
        if not self.is_controlled:
            return _messages.RejectControlMessage(
                response_code=_messages.RejectControlResponseCode.CONTROL_RELEASED)
        else:
            if self.control_available:
                if source_id == self.controlling_component:
                    self.reset_timeout()
                    self.controlling_component = None
                    return _messages.RejectControlMessage(
                        response_code=_messages.RejectControlResponseCode.CONTROL_RELEASED)
                else:
                    # if they are not the controlling client, apparently we are suposed
                    # to just ignore them...?
                    return None
            else:
                return _messages.RejectControlMessage(
                    response_code=_messages.RejectControlResponseCode.NOT_AVAILABLE)

    @_asyncio.coroutine
    def reject_control(self, source_id=None):
        if self.is_controlled:
            self.reset_timeout()
            controlling_component = self.controlling_component
            self.controlling_component = source_id
            yield from self.component.transport.send_message(
                destination_id=controlling_component,
                message=_messages.RejectControlMessage(
                    response_code=_messages.RejectControlResponseCode.CONTROL_RELEASED))

    @message_handler(
        _messages.MessageCode.QueryTimeout)
    @_asyncio.coroutine
    def on_query_timeout(self, **kwargs):
        return _messages.ReportTimeoutMessage(
            timeout=self.timeout)

    @message_handler(
        _messages.MessageCode.QueryAuthority)
    @_asyncio.coroutine
    def on_query_authority(self, **kwargs):
        return _messages.ReportAuthorityMessage(
            authority_code=self.authority)

    @message_handler(
        _messages.MessageCode.QueryControl)
    @_asyncio.coroutine
    def on_query_control(self, **kwargs):
        if self.is_controlled:
            controlling_component = self.controlling_component
        else:
            controlling_component = _messages.Id(0,0,0)

        return _messages.ReportControlMessage(
            id=controlling_component,
            authority_code=self.authority)

    @message_handler(
        _messages.MessageCode.SetAuthority,
        supports_events=False)
    @_asyncio.coroutine
    def on_set_authority(self, source_id, message, **kwargs):
        message = message.body
        # manually implement command semantics since currently
        # services can't depend on themselves
        if not self.has_control(source_id):
            return None

        authority_code = message.authority_code
        if authority_code <= self.authority and authority_code >= self.component.default_authority:
            self.authority = message.authority_code

    @property
    def control_available(self):
        return self.component.management.status in (
            _messages.ManagementStatus.READY,
            _messages.ManagementStatus.STANDBY)

class Management(Service):
    name='management'

    status = change_watcher(
        '_status',
        query_codes=(_messages.MessageCode.QueryStatus,))

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._status = _messages.ManagementStatus.STANDBY
        self.old_status = None
        self.id_store = set()

    @message_handler(
        _messages.MessageCode.Shutdown,
        is_command=True)
    @_asyncio.coroutine
    def on_shutdown(self, **kwargs):
        yield from self.component.access_control.reject_control()
        self.status = _messages.ManagementStatus.SHUTDOWN

    @message_handler(
        _messages.MessageCode.Standby,
        is_command=True)
    @_asyncio.coroutine
    def on_standby(self, **kwargs):
        if self.status is _messages.ManagementStatus.READY:
            self.status = _messages.ManagementStatus.STANDBY

    @message_handler(
        _messages.MessageCode.Resume,
        is_command=True)
    @_asyncio.coroutine
    def on_resume(self, **kwargs):
        # this is a command, so don't worry about not being controlled
        if self.status is _messages.ManagementStatus.STANDBY:
            self.status = _messages.ManagementStatus.READY

    @message_handler(
        _messages.MessageCode.Reset,
        is_command=True)
    @_asyncio.coroutine
    def on_reset(self, **kwargs):
        if self.status in (_messages.ManagementStatus.STANDBY, _messages.ManagementStatus.READY):
            yield from self.component.access_control.reject_control()
            self.status = _messages.ManagementStatus.STANDBY

    @message_handler(
        _messages.MessageCode.SetEmergency)
    @_asyncio.coroutine
    def on_set_emergency(self, source_id, **kwargs):
        self.id_store |= set(source_id)
        if self.status is not _messages.ManagementStatus.EMERGENCY:
            self.old_status = self.status
            self.status = _messages.ManagementStatus.EMERGENCY

    @message_handler(
        _messages.MessageCode.ClearEmergency)
    @_asyncio.coroutine
    def on_clear_emergency(self, source_id, **kwargs):
        self.id_store -= set(source_id)
        if not self.id_store:
            self.status = self.old_status

    @message_handler(
        _messages.MessageCode.QueryStatus)
    @_asyncio.coroutine
    def on_query_status(self, **kwargs):
        return _messages.ReportStatusMessage(
            status=self.status)

class ListManager(Service):
    name='list_manager'

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._impl = _list_manager.ListManager()

    @message_handler(
        _messages.MessageCode.QueryElement)
    @_asyncio.coroutine
    def on_query_element(self, message, **kwargs):
        element = self._impl.get(message.element_uid)
        if element is not None:
            return _messages.ReportElementMessage(
                uid=element.UID,
                previous_uid=element.previousUID,
                next_uid=element.nextUID,
                data=element.data)

    @message_handler(
        _messages.MessageCode.QueryElementList)
    @_asyncio.coroutine
    def on_query_element_list(self, **kwargs):
        return _messages.ReportElementListMessage(
            elements=[
                element.uid
                for element in _list_manager.to_list(self._impl)])

    @message_handler(
        _messages.MessageCode.QueryElementCount)
    @_asyncio.coroutine
    def on_query_element_count(self, **kwargs):
        return _messages.ReportElementCountMessage(
            element_count=self._impl.count)

    @message_handler(
        _messages.MessageCode.SetElement,
        is_command=True)
    @_asyncio.coroutine
    def on_set_element(self, message, **kwargs):
        message = message.body
        def rejection(response_code):
            return return _messages.RejectElementRequestMessage(
                request_id=message.request_id,
                response_code=response_code)
        try:
            self._impl.insert_batch([
                _list_manager.Element(next=e.next, prev=e.prev, uid=e.uid, data=e.data)
                for e in message.elements
            ])
            return _messages.ConfirmElementRequestMessage(
                request_id=message.request_id)
        except _list_manager.BrokenReference as e:
            if e.uid == e.element.next:
                return rejection(_messages.RejectEventRequestResponseCode.INVALID_NEXT_ELEMENT)
            else:
                return rejection(_messages.RejectEventRequestResponseCode.INVALID_PREVIOUS_ELEMENT)
        except _list_manager.ElementAlreadyExists:
            return rejection(_messages.RejectEventRequestResponseCode.INVALID_ELEMENT_ID)
        except _list_manager.ListError:
            return rejection(_messages.RejectEventRequestResponseCode.UNSPECIFIED_ERROR)

    @message_handler(
        _messages.MessageCode.DeleteElement,
        is_command=True)
    @_asyncio.coroutine
    def on_delete_element(self, message, **kwargs):
        message = message.body
        def rejection(response_code):
            return return _messages.RejectElementRequestMessage(
                request_id=message.request_id,
                response_code=response_code)
        try:
            self._impl.delete_batch(message.element_ids)
            return _messages.ConfirmElementRequestMessage(
                request_id=message.request_id)
        except _list_manager.BrokenReference as e:
            if e.uid == e.element.next:
                return rejection(_messages.RejectEventRequestResponseCode.INVALID_NEXT_ELEMENT)
            else:
                return rejection(_messages.RejectEventRequestResponseCode.INVALID_PREVIOUS_ELEMENT)
        except _list_manager.NoSuchElement:
            return rejection(_messages.RejectEventRequestResponseCode.INVALID_ELEMENT_ID)
        except _list_manager.ListError:
            return rejection(_messages.RejectEventRequestResponseCode.UNSPECIFIED_ERROR)

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
        pass

    @message_handler(_messages.MessageCode.QuerySubsystemList)
    @_asyncio.coroutine
    def on_query_subsystem_list():
        pass

    @message_handler(_messages.MessageCode.QueryServices)
    @_asyncio.coroutine
    def on_query_services():
        pass

    @message_handler(_messages.MessageCode.QueryServiceList)
    @_asyncio.coroutine
    def on_query_service_list():
        pass

    
    # basically there are two things to take consideration of,
    # input data via register actions
    # return answers from queries, even
    # Store all info of register actions into a table or map.
    # Handlers of event class contains info to get info, have dict, set get vals.