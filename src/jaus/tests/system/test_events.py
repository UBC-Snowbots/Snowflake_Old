import asyncio
import pytest
import time
from util import slow

import jaus.messages as _messages

@pytest.mark.asyncio
def test__report_event_timeout(connection):
    yield from connection.send_message(_messages.QueryEventTimeoutMessage())
    reply = yield from connection.receive_messages()
    assert reply == [_messages.ReportEventTimeoutMessage(timeout=1)]

@pytest.mark.asyncio
def test__report_events__no_events(connection):
    yield from connection.send_message(_messages.QueryEventsMessage(
        variant=_messages.QueryEventsVariant.ALL_EVENTS,
        all_events=0,
        message_code=None,
        event_type=None,
        event_id=None))
    reply = yield from connection.receive_messages()
    assert reply == [_messages.ReportEventsMessage(
        count=0,
        events=[]
    )]

def check_confirm_event_request(reply, request_id, rate=5, event_id=None):
    assert reply.message_code is _messages.MessageCode.ConfirmEventRequest
    msg_body = reply.body
    if event_id is not None:
        assert event_id == msg_body.event_id
    event_id = msg_body.event_id
    assert msg_body.request_id == request_id
    assert round(msg_body.confirmed_periodic_rate) == round(rate)
    return event_id

@asyncio.coroutine
def setup_event(connection, type, query, request_id, rate=5):
    yield from connection.send_message(_messages.CreateEventMessage(
        request_id=request_id,
        event_type=type,
        requested_periodic_rate=rate,
        query_message=query._serialize_to_bytes()))
    replies = yield from connection.receive_messages(
        message_count=1,
        types=(_messages.MessageCode.ConfirmEventRequest,))

    assert len(replies) == 1
    return check_confirm_event_request(
        replies[0],
        request_id=request_id,
        rate=rate)

@asyncio.coroutine
def teardown_event(connection, event_id, request_id, rate=5):
    yield from connection.send_message(_messages.CancelEventMessage(
        request_id=request_id,
        event_id=event_id))

    replies = yield from connection.receive_messages(
        types=(_messages.MessageCode.ConfirmEventRequest,))
    assert len(replies) == 1
    check_confirm_event_request(
        replies[0],
        request_id=request_id,
        event_id=event_id,
        rate=rate)

@pytest.mark.asyncio
def test__report_events__one_event(connection):
    event_id = yield from setup_event(
        connection,
        type=_messages.EventType.PERIODIC,
        query=_messages.QueryHeartbeatPulseMessage(),
        request_id=1)

    yield from connection.send_message(_messages.QueryEventsMessage(
        variant=_messages.QueryEventsVariant.ALL_EVENTS,
        all_events=0,
        message_code=None,
        event_type=None,
        event_id=None))

    replies = yield from connection.receive_messages(
        types=(_messages.MessageCode.ReportEvents,))
    assert replies == [_messages.ReportEventsMessage(
        count=1,
        events=[_messages.ReportEvent(
            event_type=_messages.EventType.PERIODIC,
            event_id=event_id,
            query_message=_messages.QueryHeartbeatPulseMessage()._serialize_to_bytes())])]

    yield from teardown_event(connection, event_id=event_id, request_id=2)

@slow
@pytest.mark.asyncio
def test__event_rate(connection):
    rate = 5
    event_id = yield from setup_event(
        connection,
        type=_messages.EventType.PERIODIC,
        query=_messages.QueryHeartbeatPulseMessage(),
        request_id=1,
        rate=rate)

    results = []
    for i in range(20):
        start_time = time.clock()
        reports = yield from connection.receive_messages(
            message_count=1,
            types=(_messages.MessageCode.Event,))
        results.append(time.clock() - start_time)

    assert round(sum(results)/20, 1) == round(1/5, 1)

    yield from teardown_event(
        connection,
        event_id=event_id,
        request_id=2,
        rate=rate)

@slow
@pytest.mark.asyncio
def test__event_timeout(connection):
    rate = 5
    event_id = yield from setup_event(
        connection,
        type=_messages.EventType.PERIODIC,
        query=_messages.QueryHeartbeatPulseMessage(),
        request_id=1,
        rate=rate)

    results = yield from connection.receive_messages(
        message_count=1,
        types=(_messages.MessageCode.ConfirmEventRequest,),
        timeout=75)
    assert len(results) == 1
    check_confirm_event_request(results[0], request_id=1, event_id=event_id, rate=5)
