import logging

from twisted.web import server
from twisted.internet import reactor, endpoints
from . import Test
from . import JAUSProtocol

def setup_logging():
	logging.basicConfig(level=logging.DEBUG)

def test_twisted():
	setup_logging()
	endpoints.serverFromString(reactor, "tcp:8080").listen(server.Site(Test()))
	reactor.run()

def test_jaus():
	setup_logging()
	reactor.listenUDP(9999, JAUSProtocol())
	reactor.run()
