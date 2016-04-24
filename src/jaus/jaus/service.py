import asyncio as _asyncio

import jaus.messages as _messages
import jaus.judp as _judp
import jaus.signal as _signal

class Service:
    state_changed = _signal.Signal(key='new_state')

    def __init__(self, component, state=None):
        self.component = component
        self._state = state

    def on_message(self, message_code):
        def register(fn):
            return self.component.message_received.connect(
              fn, message_code=message_code)
        return register

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, state):
        old_state = self._state
        self._state = state

        self.state_changed.send(
          new_state=self._state,
          old_state=old_state)

class Component(object):
    def __init__(self, id, services = set()):
        self.id = id
        self.services = {}
        self.message_received = _signal.Signal(key='message_code')

        for service in services:
            instance = service(component=self)
            self.services[instance.name] = instance

        @_asyncio.coroutine
        def message_handler(**kwargs):
            for result in self.message_received.send(**kwargs):
                yield from result
        self.transport.message_received.connect(message_handler)

    def __getattr__(self, name):
        if name in self.services:
            return self.services[name]
        else:
            raise AttributeError(name)

class Transport(Service):
    name='transport'

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        message_received = _signal.Signal(key='message_code')
        self.protocol = _judp.JUDPProtocol(
            message_received_signal=message_received,
            own_id=self.component.id)
        self.message_received = message_received

    @_asyncio.coroutine
    def send_message(self, *args, **kwargs):
        yield from self.protocol.send_message(*args, **kwargs)

class Liveness(Service):
    name='liveness'

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        @self.on_message(_messages.MessageCode.QueryHeartbeatPulse)
        @_asyncio.coroutine
        def on_query_heartbeat(source_id, **kwargs):
            yield from self.component.transport.send_message(
                destination_id=source_id,
                message=_messages.ReportHeartbeatPulseMessage())
