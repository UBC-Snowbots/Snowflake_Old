import ctypes as _ctypes

# TODO: actuall implement

_MESSAGES = {}

def message_type(type_id):
	def register_fn(message_class):
		_MESSAGES[type_id] = message_class
	return register_fn

def Message(packets):
	return "foo"
