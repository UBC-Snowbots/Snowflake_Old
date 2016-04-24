import pytest
import bitstring

from unittest.mock import create_autospec, sentinel, call, ANY, Mock

from jaus.format import (
    Consume,
    Group,
    Int,
    Optional,
    Spec,
    Specification,
)


class TestGroup(object):
    @pytest.fixture
    def specs(self):
        return [
          create_autospec(Spec, instance=True),
          create_autospec(Spec, instance=True),
        ]

    def test_read_basic(self, specs):
        g = Group(specs=specs)
        buf = create_autospec(bitstring.BitStream, instance=True)

        specs[0].own_names = [sentinel.foo, sentinel.bar]
        specs[0].read.return_value = {
          sentinel.foo: sentinel.foo_v,
          sentinel.bar: sentinel.bar_v,
        }
        specs[1].own_names = [sentinel.ping]
        specs[1].read.return_value = {
          sentinel.ping: sentinel.ping_v,
        }

        assert g.own_names == [sentinel.foo, sentinel.bar, sentinel.ping]

        assert g.read(buf, {}) == {
          sentinel.foo: sentinel.foo_v,
          sentinel.bar: sentinel.bar_v,
          sentinel.ping: sentinel.ping_v,
        }

        assert specs[0].read.call_args_list == [
          call(buf, ANY),
        ]
        assert specs[1].read.call_args_list == [
          call(buf, ANY),
        ]

class TestInt(object):
    def test_invalid_input(self):
        i = Int('test', bits=4)

        with pytest.raises(bitstring.ReadError):
            i.read(bitstring.BitStream(bin='10'))

    def test_invalid_constructor_params(self):
        with pytest.raises(ValueError):
            Int('test', bits=12, endianness='le')
            with pytest.raises(ValueError):
                Int('test', endianness='incorrect')

class TestConsume(object):
    pass

class TestFuzz(object):
    def test_simple_specification(self):
        spec = Specification('Test', [
                Int('foo', bits=3),
                Int('bar', bits=5),
                Int('ping', bytes=2, endianness='le'),
            ])

        instance = spec.instantiate(bitstring.BitStream(
                bin=(
                    '010'
                    '10110'
                    '11010010' '01011010')))

        assert bin(instance.foo) == bin(0b010)
        assert bin(instance.bar) == bin(0b10110)
        assert bin(instance.ping) == bin(0b0101101011010010)

    def test_basic_consume(self):
        spec = Specification('Test', [
                Int('foo', bits=2),
                Int('bar', bits=5),
            ])
        tp = spec.type
        c = Consume('test', spec)

        assert c.read(bitstring.BitStream(bin=(
                    '10' '01001'
                    '01' '10110'))) == {
            'test': [
                tp(foo=0b10, bar=0b01001),
                tp(foo=0b01, bar=0b10110),
            ]
        }

        assert c.read(bitstring.BitStream(bin='')) == {
            'test': []
        }

    def test_consume_size_not_multiple_of_spec(self):
        spec = Specification('Test', [
                Int('foo', bits=2),
                Int('bar', bits=5),
            ])
        tp = spec.type
        c = Consume('test', spec)

        with pytest.raises(bitstring.ReadError):
            c.read(bitstring.BitStream(bin=(
                        '10' '01001'
                        '1001')))

    def test_optional(self):
        pred = Mock(name='test_predicate')
        spec = Specification('Test', [
                Optional(pred, [
                        Int('test', bytes=1),
                    ]),
            ])

        pred.return_value = True
        assert spec.instantiate(
            bitstring.BitStream(hex='0xAB')) == spec.type(
            test=0xAB)
        assert pred.call_args == call({
                'test': 0xAB
            })

        pred.return_value = False
        assert spec.instantiate(
            bitstring.BitStream(hex='0xAB')) == spec.type(
            test=None)
        assert pred.call_args == call({
                'test': None
            })
