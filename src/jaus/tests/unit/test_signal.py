import pytest
from unittest.mock import sentinel

import jaus.signal as signal

def test__signal_basic_dispatch():
    sig = signal.Signal(key='a')

    @sig.connect(a=1)
    def callback(a, b):
        return a, b

    assert sig.send(a=1, b=2) == [(1, 2)]
    assert sig.send(a=2, b=1) == []
    assert sig.send() == []
    assert sig.send(a=3) == []

def test__signal_no_properties():
    sig = signal.Signal()

    @sig.connect
    def callback():
        return True

    assert sig.send() == [True]
    with pytest.raises(TypeError):
        sig.send(a=3)
