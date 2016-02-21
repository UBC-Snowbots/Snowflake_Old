import logging

from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from twisted.web import resource

class Test(resource.Resource):
	isLeaf = True
	
	def render_GET(self, request):
		request.setHeader(b"content-type", b"text/plain")
		return "test".encode("ascii")

class JAUSProtocol(DatagramProtocol):
	def __init__(self):
		self.log = logging.getLogger("{}.JAUSProtocol".format(__name__))
	def datagramReceived(self, data, host_port_pair):
		host, port = host_port_pair
		self.log.info(
			"Received {data} from {host}:{port}"
			.format(
				data=data,
				host=host,
				port=port))
