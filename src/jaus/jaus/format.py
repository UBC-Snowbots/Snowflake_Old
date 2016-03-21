import abc as _abc
import bitstring as _bitstring
import collections as _collections
import math as _math

from chainmap import ChainMap as _ChainMap


class Spec(object):
	__metaclass__ = _abc.ABCMeta
	@_abc.abstractmethod
	def read(self, buf, attributes={}):
		"""Read one or more attributes from ``buf``.
		
		:param buf BitStream:
		
		Return a dictionary of these attributes.
		May require certain attributes to exist in
		``attributes`` in order to function.
		"""
	@_abc.abstractmethod
	def write(self, buf, attributes):
		"""Write any owned properties in ``attributes`` to ``buf``.
			
		:param buf BitStream
		"""
	
	@property
	@_abc.abstractmethod
	def own_names(self):
		return []

class Group(Spec):
	def __init__(self, specs):
		self.specs = specs
	
	@property
	def own_names(self):
		return [
			name
			for spec in self.specs
			for name in spec.own_names]
	
	def read(self, buf, attributes={}):
		new_attributes = {}
		combined_attributes = _ChainMap(
			new_attributes, attributes)
		for spec in self.specs:
			attrs = spec.read(buf, combined_attributes)
			new_attributes.update(attrs)
		return new_attributes
	
	def write(self, buf, attributes):
		for spec in self.specs:
			spec.write(buf, attributes)

class Specification(Group):
	def __init__(self, name, specs):
		super(Specification, self).__init__(specs)
		self.name = name
		self.type = _collections.namedtuple(name, self.own_names)
		
	def instantiate(self, buf):
		return self.type(**self.read(buf))
	
	def serialize(self, instance):
		buf = _bitstring.BitStream()
		self.write(buf, instance._asdict())
		return buf

class Optional(Group):
	def __init__(self, condition, specs):
		super(Optional, self).__init__(specs)
		self.condition = condition
	def read(self, buf, attributes={}):
		if self.condition(attributes):
			return super(Optional, self).read(buf, attributes)
		else:
			return {name: None for name in self.own_names}
	def write(self, buf, attributes):
		if self.condition(attributes):
			super(Optional, self).write(buf, attributes)

class NamedSpec(Spec):
	def __init__(self, name):
		self.name = name
	
	def read(self, buf, attributes={}):
		return {
			self.name: self.read_one(buf, attributes)
		}
	
	def write(self, buf, attributes):
		self.write_one(buf, attributes[self.name])
	
	@property
	def own_names(self):
		return [self.name]
	
	@_abc.abstractmethod
	def read_one(self, buf, attributes):
		pass
	
	@_abc.abstractmethod
	def write_one(self, buf, prop):
		pass

class Consume(NamedSpec):
	def __init__(self, name, specification):
		super(Consume, self).__init__(name)
		self.specification = specification
	def read_one(self, buf, attributes):
		result = []
		while buf.pos < buf.len:
			result.append(self.specification.instantiate(buf))
		return result
	def write_one(self, buf, prop):
		for p in prop:
			self.specification.serialize(buf, p)

class Int(NamedSpec):
	def __init__(self, name, bits=0, bytes=0, endianness=None):
		super(Int, self).__init__(name)
		self.bits = bits + bytes*8
		self.endianness = endianness
		if endianness not in (None, 'le', 'be'):
			raise ValueError('endianness "{}" is not one of None, '
				'"le" or "be".'.format(endianness))
		if endianness is not None and self.bits % 8:
			raise ValueError(
				'Endianness set on int with non-whole '
				'number of bytes: {}'.format(self.bits))
	def _get_type_string(self):
		if self.endianness is not None:
			return 'uint{}'.format(self.endianness)
		else:
			return 'uint'
	def read_one(self, buf, attributes):
		return buf.read('{}:{}'.format(
			self._get_type_string(), self.bits))
	def write_one(self, buf, prop):
		buf.append(_bitstring.BitArray(**{
			'length': self.bits,
			self._get_type_string(): prop
		}))

class Enum(Int):
	def __init__(self, name, enum, *args, **kwargs):
		super(Enum, self).__init__(name, *args, **kwargs)
		self.enum = enum
	def read_one(self, buf, attributes):
		return self.enum(
			super(Enum, self).read_one(buf, attributes))
	def write_one(self, buf, prop):
		super(Enum, self).write_one(buf, prop.value)

class Bytes(NamedSpec):
	def __init__(self, name, length):
		super(Bytes, self).__init__(name)
		self.length = length
	def read_one(self, buf, attributes):
		length = self.length
		if callable(length):
			length = length(attributes)
		return buf.read('bytes:{}'.format(length))
	def write_one(self, buf, prop):
		buf.append(_bitstring.BitArray(bytes=prop))
