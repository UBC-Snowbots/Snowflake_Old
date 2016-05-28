import asyncio
import pytest

import jaus.messages as messages
from util import slow

@pytest.mark.asyncio
def test__query_control__not_controlled(connection):
    yield from connection.send_message(
        messages.QueryControlMessage())

    results = yield from connection.receive_messages(types=(messages.MessageCode.ReportControl,))
    assert results == [messages.ReportControlMessage(
        id=messages.Id(0,0,0),
        authority_code=0)]

@pytest.mark.asyncio
def test__query_control__controlled(control_connection, test_id):
    connection = control_connection

    yield from connection.send_message(
        messages.QueryControlMessage())

    results = yield from connection.receive_messages(types=(messages.MessageCode.ReportControl,))
    assert results == [messages.ReportControlMessage(
        id=test_id,
        authority_code=5)]

@pytest.mark.asyncio
def test__preemption__insufficient_authority(control_connection, connection2):
    yield from connection2.send_message(messages.RequestControlMessage(
        authority_code=4))
    results = yield from connection2.receive_messages(types=(messages.MessageCode.ConfirmControl,))

    assert results == [messages.ConfirmControlMessage(
        response_code=messages.ConfirmControlResponseCode.INSUFFICIENT_AUTHORITY)]

@asyncio.coroutine
def reacquire_control(connection):
    yield from connection.send_message(messages.RequestControlMessage(
        authority_code=5))
    results = yield from connection.receive_messages(types=(messages.MessageCode.ConfirmControl,))
    assert results == [messages.ConfirmControlMessage(
        response_code=messages.ConfirmControlResponseCode.CONTROL_ACCEPTED)]

@asyncio.coroutine
def release_control(connection):
    yield from connection.send_message(messages.ReleaseControlMessage())
    results = yield from connection.receive_messages(types=(messages.MessageCode.RejectControl,))
    assert results == [messages.RejectControlMessage(
        response_code=messages.RejectControlResponseCode.CONTROL_RELEASED)]

@pytest.mark.asyncio
def test__preemption__sufficient_authority(control_connection, connection2):
    yield from connection2.send_message(messages.RequestControlMessage(
        authority_code=6))
    results = yield from connection2.receive_messages(types=(messages.MessageCode.ConfirmControl,))
    assert results == [messages.ConfirmControlMessage(
        response_code=messages.ConfirmControlResponseCode.CONTROL_ACCEPTED)]

    results = yield from control_connection.receive_messages(types=(messages.MessageCode.RejectControl,))
    assert results == [messages.RejectControlMessage(
        response_code=messages.RejectControlResponseCode.CONTROL_RELEASED)]

    yield from release_control(connection2)
    yield from reacquire_control(control_connection)

@slow
@pytest.mark.asyncio
def test__control_timeout(control_connection):
    # wait for timeout_routine
    results = yield from control_connection.receive_messages(
        types=(messages.MessageCode.RejectControl,), timeout=6)
    assert results == [messages.RejectControlMessage(
        response_code=messages.RejectControlResponseCode.CONTROL_RELEASED)]

    yield from reacquire_control(control_connection)

@pytest.mark.asyncio
def test__query_timeout(connection):
    yield from connection.send_message(
        messages.QueryTimeoutMessage())

    results = yield from connection.receive_messages(types=(messages.MessageCode.ReportTimeout,))
    assert results == [messages.ReportTimeoutMessage(timeout=5)]

@pytest.mark.asyncio
def test__query_authority(control_connection):
    connection = control_connection
    yield from connection.send_message(
        messages.QueryAuthorityMessage())

    results = yield from connection.receive_messages(types=(messages.MessageCode.ReportAuthority,))
    assert results == [messages.ReportAuthorityMessage(authority_code=5)]

@pytest.mark.parametrize('code,response', [
    (0, 0),
    (3, 0),
    (5, 0),
    (10, 0),
])
@pytest.mark.asyncio
def test__set_authority__no_control(connection, code, response):
    yield from connection.send_message(
        messages.SetAuthorityMessage(authority_code=code))

    yield from connection.send_message(
        messages.QueryAuthorityMessage())
    results = yield from connection.receive_messages(types=(messages.MessageCode.ReportAuthority,))
    assert results == [messages.ReportAuthorityMessage(authority_code=response)]

@pytest.mark.parametrize('code,response', [
    (0, 0),
    (3, 3),
    (5, 5),
    (10, 5),
])
@pytest.mark.asyncio
def test__set_authority__control(control_connection, code, response):
    connection = control_connection
    yield from connection.send_message(
        messages.SetAuthorityMessage(authority_code=code))

    yield from connection.send_message(
        messages.QueryAuthorityMessage())
    results = yield from connection.receive_messages(types=(messages.MessageCode.ReportAuthority,))
    assert results == [messages.ReportAuthorityMessage(authority_code=response)]
