import abc as _abc
import bitstring as _bitstring
import collections as _collections

def defaultnamedtuple(name, fields, defaults):
    superclass = _collections.namedtuple(name, fields)

    def __new__(cls, *args, **kwargs):
        properties = defaults.copy()
        properties.update(kwargs)
        return super(subclass, cls).__new__(
            cls, *args, **properties)
    def _asdict(self):
        return _collections.OrderedDict(
            zip(self._fields, self))

    subclass = type(name, (superclass,), {
        '__new__': __new__,
        '_asdict': _asdict,
    })
    return subclass

class Spec(metaclass=_abc.ABCMeta):
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

    @_abc.abstractmethod
    def size(self, attributes):
        """Predict the size in bits of ``attributes`` written to a buffer."""
        pass

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
        combined_attributes = _collections.ChainMap(
                new_attributes, attributes)
        for spec in self.specs:
            try:
                attrs = spec.read(buf, combined_attributes)
            except _bitstring.ReadError:
                raise
            new_attributes.update(attrs)
        return new_attributes

    def write(self, buf, attributes):
        for spec in self.specs:
            spec.write(buf, attributes)

    def size(self, attributes):
        return sum(spec.size(attributes) for spec in self.specs)

class Specification(Group):
    def __init__(self, name, specs, defaults={}):
        super(Specification, self).__init__(specs)
        self.name = name
        self.type = defaultnamedtuple(
            name, self.own_names, defaults)

    def instantiate(self, buf):
        return self.type(**self.read(buf))

    def serialize_to_bytes(self, *args, **kwargs):
        return self.serialize(*args, **kwargs).bytes

    def serialize(self, instance):
        buf = _bitstring.BitStream()
        self.write(buf, instance._asdict())
        return buf

    def size_instance(self, instance):
        return self.size(instance._asdict())

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
    def size(self, attributes):
        if self.condition(attributes):
            return super(Optional, self).size(attributes)
        else:
            return 0

class NamedSpec(Spec):
    def __init__(self, name):
        self.name = name

    def read(self, buf, attributes={}):
        value = self.read_one(buf, attributes)
        return {
                self.name: value
        }

    def write(self, buf, attributes):
        self.write_one(buf, attributes[self.name], attributes)

    def size(self, attributes):
        return self.size_one(attributes[self.name], attributes)

    @property
    def own_names(self):
        return [self.name]

    @_abc.abstractmethod
    def read_one(self, buf, attributes):
        pass

    @_abc.abstractmethod
    def write_one(self, buf, prop, attributes):
        pass

    @_abc.abstractmethod
    def size_one(self, attribute, attributes):
        pass

class Switch(NamedSpec):
    def __init__(self, name, spec_generator):
        super(Switch, self).__init__(name)
        self.spec_generator = spec_generator
    def read_one(self, buf, attributes):
        specification = self.spec_generator(attributes)
        return specification.instantiate(buf)
    def write_one(self, buf, prop, attributes):
        specification = self.spec_generator(attributes)
        buf.append(specification.serialize(prop))
    def size_one(self, attribute, attributes):
        specification = self.spec_generator(attributes)
        return specification.size_instance(attribute)

class Consume(NamedSpec):
    def __init__(self, name, specification):
        super(Consume, self).__init__(name)
        self.specification = specification
    def read_one(self, buf, attributes):
        result = []
        while buf.pos < buf.len:
            result.append(self.specification.instantiate(buf))
        return result
    def write_one(self, buf, prop, _):
        for p in prop:
            buf.append(self.specification.serialize(p))
    def size_one(self, attribute, _):
        return sum(
                self.specification.size_instance(a) for a in attribute)

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
    def write_one(self, buf, prop, _):
        buf.append(_bitstring.BitArray(**{
                'length': self.bits,
                self._get_type_string(): prop
        }))
    def size_one(self, attribute, _):
        return self.bits

class Enum(Int):
    def __init__(self, name, enum, *args, **kwargs):
        super(Enum, self).__init__(name, *args, **kwargs)
        self.enum = enum
    def read_one(self, buf, attributes):
        return self.enum(
                super(Enum, self).read_one(buf, attributes))
    def write_one(self, buf, prop, _):
        super(Enum, self).write_one(buf, prop.value, _)

class Bytes(NamedSpec):
    def __init__(self, name, length):
        super(Bytes, self).__init__(name)
        self.length = length
    def _get_length(self, attributes):
        length = self.length
        if callable(length):
            length = length(attributes)
        return length
    def read_one(self, buf, attributes):
        length = self._get_length(attributes)
        return buf.read('bytes:{}'.format(length))
    def write_one(self, buf, prop, _):
        buf.append(_bitstring.BitArray(bytes=prop))
    def size_one(self, prop, _):
        return len(prop)*8
