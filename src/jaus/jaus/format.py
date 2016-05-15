import abc as _abc
import bitstring as _bitstring
import collections as _collections

def defaultnamedtuple(name, fields, defaults):
    """Generate a defaultnamedtuple.

    defaultnamedtuple aims to remote unnecessary parameters
    required in namedtuple instantiations by allowing
    programmers to define defaults that are filled if not
    provided by the user.

    As an extra feature, if a default value is callable it
    will be called to generate the actual value used.

    This can be helpful for generating length fields based
    on the size of another attribute that was provided.
    """
    superclass = _collections.namedtuple(name, fields)

    def __new__(cls, *args, **kwargs):
        properties = defaults.copy()
        properties.update(kwargs)
        properties = {
            k: v(properties) if callable(v) else v
            for k,v in properties.items()
        }
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

    @property
    @_abc.abstractmethod
    def own_names(self):
        return []

def specification(name, specs, defaults={}):
    """Generate a specification.

    A specification is an end user's interface to the format module.
    It uses a Group to generate a set of properties
    for itself and can be transparently used like a namedtuple.

    It provides the extra features found in defaultnamedtuple,
    as well as methods to convert to and from bitstreams.
    """
    _group = Group(specs)
    base_class = defaultnamedtuple(
            name, _group.own_names, defaults)

    @classmethod
    def _instantiate(cls, buf):
        return cls(**cls._group.read(buf))

    def _serialize_to_bytes(self, *args, **kwargs):
        return self._serialize(*args, **kwargs).bytes

    def _serialize(self):
        buf = _bitstring.BitStream()
        _group.write(buf, self._asdict())
        return buf

    return type(name, (base_class,), {
        '_instantiate': _instantiate,
        '_serialize_to_bytes': _serialize_to_bytes,
        '_serialize': _serialize,
        '_group': _group,
    })

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

class Optional(Group):
    def __init__(self, condition, specs):
        super().__init__(specs)
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
        value = self.read_one(buf, attributes)
        return {
                self.name: value
        }

    def write(self, buf, attributes):
        self.write_one(buf, attributes[self.name], attributes)

    @property
    def own_names(self):
        return [self.name]

    @_abc.abstractmethod
    def read_one(self, buf, attributes):
        pass

    @_abc.abstractmethod
    def write_one(self, buf, prop, attributes):
        pass

class Switch(NamedSpec):
    def __init__(self, name, spec_generator):
        super().__init__(name)
        self.spec_generator = spec_generator
    def read_one(self, buf, attributes):
        specification = self.spec_generator(attributes)
        return specification._instantiate(buf)
    def write_one(self, buf, prop, attributes):
        specification = self.spec_generator(attributes)
        buf.append(prop._serialize())

class Instance(NamedSpec):
    def __init__(self, name, specification):
        super().__init__(name)
        self.specification = specification
    def read_one(self, buf, attributes):
        return self.specification._instantiate(buf)
    def write_one(self, buf, prop, _):
        buf.append(prop._serialize())

class Consume(NamedSpec):
    def __init__(self, name, specification):
        super().__init__(name)
        self.specification = specification
    def read_one(self, buf, attributes):
        result = []
        while buf.pos < buf.len:
            result.append(self.specification._instantiate(buf))
        return result
    def write_one(self, buf, prop, _):
        for p in prop:
            buf.append(p._serialize())

class Repeat(NamedSpec):
    def __init__(self, name, specification, count):
        super().__init__(name)
        self.specification = specification
        self.count = count
    def read_one(self, buf, attributes):
        count = self.count
        if callable(count):
            count = count(attributes)
        return [self.specification._instantiate(buf) for i in range(count)]
    def write_one(self, buf, prop, _):
        for p in prop:
            buf.append(p._serialize())

class Int(NamedSpec):
    def __init__(self, name, bits=0, bytes=0, endianness=None, unsigned=True):
        super().__init__(name)
        self.bits = bits + bytes*8
        self.endianness = endianness
        self.unsigned = unsigned
        if endianness not in (None, 'le', 'be'):
            raise ValueError('endianness "{}" is not one of None, '
                    '"le" or "be".'.format(endianness))
        if endianness is not None and self.bits % 8:
            raise ValueError(
                    'Endianness set on int with non-whole '
                    'number of bytes: {}'.format(self.bits))
    @property
    def max(self):
        return (2**self.bits) - 1
    def _get_type_string(self):
        return '{unsigned}int{endianness}'.format(
            unsigned='u' if self.unsigned else '',
            endianness=self.endianness if self.endianness is not None else '')
    def read_one(self, buf, attributes):
        return buf.read('{}:{}'.format(
                self._get_type_string(), self.bits))
    def write_one(self, buf, prop, _):
        buf.append(_bitstring.BitArray(**{
                'length': self.bits,
                self._get_type_string(): prop
        }))

class Enum(Int):
    def __init__(self, name, enum, *args, **kwargs):
        super().__init__(name, *args, **kwargs)
        self.enum = enum
    def read_one(self, buf, attributes):
        return self.enum(
                super(Enum, self).read_one(buf, attributes))
    def write_one(self, buf, prop, _):
        super(Enum, self).write_one(buf, prop.value, _)

class Bytes(NamedSpec):
    def __init__(self, name, length):
        super().__init__(name)
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
