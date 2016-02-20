from twisted.web import resource

class Test(resource.Resource):
	isLeaf = True
	
	def render_GET(self, request):
		request.setHeader(b"content-type", b"text/plain")
		return "test".encode("ascii")
