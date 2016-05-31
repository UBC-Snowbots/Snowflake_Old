import pytest
from unittest.mock import sentinel

from jaus.list_manager import ListManager, Element, ListError, to_list

def test__single_element():
    lm = ListManager()
    a = Element(next=0, prev=0, data=sentinel.a, uid=1)

    lm[1] = a
    assert to_list(lm) == [a]

    with pytest.raises(ListError):
        lm[1] = a

    del lm[1]
    assert to_list(lm) == []

    with pytest.raises(KeyError):
        lm[2]

def test__two_elements():
    lm = ListManager()
    a = Element(next=2, prev=0, data=sentinel.a, uid=1)
    b = Element(next=0, prev=1, data=sentinel.b, uid=2)
    lm.insert_batch([a, b])

    assert to_list(lm) == [
        Element(next=2, prev=0, data=sentinel.a, uid=1),
        Element(next=0, prev=1, data=sentinel.b, uid=2),
    ]

    with pytest.raises(ListError):
        lm.insert_batch([a])

    del lm[1]
    assert to_list(lm) == [
        Element(next=0, prev=0, data=sentinel.b, uid=2),
    ]

def test__cycle():
    lm = ListManager()
    a = Element(next=2, prev=0, data=sentinel.a, uid=1)
    b = Element(next=3, prev=1, data=sentinel.b, uid=2)
    c = Element(next=2, prev=2, data=sentinel.c, uid=3)
    lm.insert_batch([a,b,c])

    assert to_list(lm) == [
        Element(prev=0, next=2, uid=1, data=sentinel.a),
        Element(prev=1, next=3, uid=2, data=sentinel.b),
        Element(prev=2, next=2, uid=3, data=sentinel.c),
    ]

    del lm[3]
    assert to_list(lm) == [
        Element(prev=0, next=2, uid=1, data=sentinel.a),
        Element(prev=1, next=0, uid=2, data=sentinel.b),
    ]
