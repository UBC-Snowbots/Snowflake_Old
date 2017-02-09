import pytest

import jaus.messages as messages

@pytest.mark.asyncio(forbid_global_loop=True)
def test__query_status__standby(control_connection):
    yield from control_connection.send_message(
        messages.QueryStatusMessage())

    results = yield from control_connection.receive_messages()
    assert results == [messages.ReportStatusMessage(
        status=messages.ManagementStatus.STANDBY)]

@pytest.mark.asyncio(forbid_global_loop=True)
def test__set_emergency_and_restore(control_connection):
    yield from control_connection.send_message(
        messages.SetEmergencyMessage(emergency_code=messages.EmergencyCode.STOP))

    yield from control_connection.send_message(
        messages.QueryStatusMessage())
    results = yield from control_connection.receive_messages()
    assert results == [messages.ReportStatusMessage(
        status=messages.ManagementStatus.EMERGENCY)]

    yield from control_connection.send_message(
        messages.ClearEmergencyMessage(emergency_code=messages.EmergencyCode.STOP))

    yield from control_connection.send_message(
        messages.QueryStatusMessage())
    results = yield from control_connection.receive_messages()
    assert results == [messages.ReportStatusMessage(
        status=messages.ManagementStatus.STANDBY)]

@pytest.mark.asyncio(forbid_global_loop=True)
def test__set_emergency__not_controlled__denies_control(connection):
    yield from connection.send_message(
        messages.SetEmergencyMessage(emergency_code=messages.EmergencyCode.STOP))

    yield from connection.send_message(
        messages.RequestControlMessage(authority_code=5))
    results = yield from connection.receive_messages()
    assert results == [
        messages.ConfirmControlMessage(response_code=messages.ConfirmControlResponseCode.NOT_AVAILABLE)]

    yield from connection.send_message(
        messages.ClearEmergencyMessage(emergency_code=messages.EmergencyCode.STOP))

@pytest.mark.asyncio(forbid_global_loop=True)
def test__set_emergency__controlled__denies_control_and_release(control_connection):
    connection = control_connection
    yield from connection.send_message(
        messages.SetEmergencyMessage(emergency_code=messages.EmergencyCode.STOP))

    yield from connection.send_message(
        messages.RequestControlMessage(authority_code=5))
    results = yield from connection.receive_messages()
    assert results == [
        messages.ConfirmControlMessage(response_code=messages.ConfirmControlResponseCode.NOT_AVAILABLE)]

    yield from connection.send_message(
        messages.ReleaseControlMessage())
    results = yield from connection.receive_messages()
    assert results == [
        messages.RejectControlMessage(response_code=messages.RejectControlResponseCode.NOT_AVAILABLE)]

    yield from connection.send_message(
        messages.ClearEmergencyMessage(emergency_code=messages.EmergencyCode.STOP))
