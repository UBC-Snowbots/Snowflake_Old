import collections as _collections

class Signal:
    def __init__(self, key=None):
        self.dispatch_table = _collections.defaultdict(
            lambda: set())
        self.key = key

    def connect(self, fn=None, **kwargs):
        def connect(fn):
            if self.key in kwargs:
                self.dispatch_table[kwargs[self.key]].add(fn)
            else:
                self.dispatch_table[None].add(fn)
        if fn is None:
            return connect
        else:
            return connect(fn)

    def send(self, **kwargs):
        callbacks = self.dispatch_table[None]
        if self.key in kwargs:
            callbacks = (
                callbacks
                | self.dispatch_table[kwargs[self.key]])
        return [
            callback(**kwargs)
            for callback in callbacks]
