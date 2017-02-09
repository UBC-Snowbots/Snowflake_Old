import pytest
import jaus.messages as _messages

multicast_addr = ('239.255.0.1', 3794)

@pytest.mark.asyncio(forbid_global_loop=True)
def test__identification__multicast(connection_multicast):
    connection = connection_multicast
    yield from connection.send_message(_messages.QueryIdentificationMessage(
        type=_messages.IdentificationQueryType.COMPONENT), addr=multicast_addr)

    results = yield from connection.receive_messages(message_count=2)
    assert set(results) == {
        _messages.ReportIdentificationMessage(
            query_type=_messages.IdentificationQueryType.COMPONENT,
            type=_messages.IdentificationType.COMPONENT,
            identification='Core'),
        _messages.ReportIdentificationMessage(
            query_type=_messages.IdentificationQueryType.COMPONENT,
            type=_messages.IdentificationType.COMPONENT,
            identification='Mobility'),
    }

@pytest.mark.asyncio(forbid_global_loop=True)
def test__identification__query_services(connection, test_id):
    yield from connection.send_message(_messages.RegisterServicesMessage(
        services=[
            _messages.Service(uri='foobinator', major_version=3, minor_version=2),
            _messages.Service(uri='barinator', major_version=0, minor_version=1)
        ]))
    yield from connection.send_message(_messages.QueryServicesMessage(
        nodes=[
            _messages.NodeRequest(
                id=25,
                components=[
                    _messages.ComponentRequest(id=2),
                ]),
            _messages.NodeRequest(
                id=1,
                components=[
                    _messages.ComponentRequest(id=1),
                ]),
        ]))
    results = yield from connection.receive_messages()
    assert len(results) == 1
    nodes = {}
    for node in results[0].body.nodes:
        nodes[node.id] = {
            component.id: set(component.services)
            for component in node.components
        }
    assert nodes == {
        1: {
            1: {
                _messages.Service(uri='urn:jaus:jss:core:Transport', major_version=1, minor_version=0),
                _messages.Service(uri='urn:jaus:jss:core:Events', major_version=1, minor_version=0),
                _messages.Service(uri='urn:jaus:jss:core:AccessControl', major_version=1, minor_version=0),
                _messages.Service(uri='urn:jaus:jss:core:Management', major_version=1, minor_version=0),
                _messages.Service(uri='urn:jaus:jss:core:Transport', major_version=1, minor_version=0),
                _messages.Service(uri='urn:jaus:jss:core:Events', major_version=1, minor_version=0),
                _messages.Service(uri='urn:jaus:jss:core:Liveness', major_version=1, minor_version=0),
                _messages.Service(uri='urn:jaus:jss:core:Discovery', major_version=1, minor_version=0),
                _messages.Service(uri='urn:jaus:jss:core:ListManager', major_version=1, minor_version=0),
            }
        },
        25: {
            2: set(),
        },
    }

    yield from connection.send_message(_messages.QueryServiceListMessage(
        subsystems=[
            _messages.SubsystemListRequest(
                id=test_id.subsystem,
                nodes=[
                    _messages.NodeListRequest(
                        id=test_id.node,
                        components=[
                            _messages.ComponentListRequest(
                                id=test_id.component),
                        ]),
                ])
        ]))
    results = yield from connection.receive_messages()
    subsystems = {}
    for subsystem in results[0].body.subsystems:
        subsystems[subsystem.id] = {
            node.id: {
                component.id: set(component.services)
                for component in node.components
            }
            for node in subsystem.nodes
        }
    assert subsystems == {
        test_id.subsystem: {
            test_id.node: {
                test_id.component: {
                    _messages.Service(uri='foobinator', major_version=3, minor_version=2),
                    _messages.Service(uri='barinator', major_version=0, minor_version=1)
                }
            }
        }
    }
