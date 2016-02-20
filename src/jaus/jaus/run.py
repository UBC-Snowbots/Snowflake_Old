from twisted.web import server
from twisted.internet import reactor, endpoints
from . import Test

def test_twisted():
	endpoints.serverFromString(reactor, "tcp:8080").listen(server.Site(Test()))
	reactor.run()
