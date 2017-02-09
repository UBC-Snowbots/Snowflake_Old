import collections as _collections

class ListElement = _collections.namedtuple('Element', ['prev', 'uid', 'next'])

class LinkedListIterator(_collections.abc.Iterator):
    def __init__(self, listmanager):
        self._listmanager = listmanager
        self._pos = listmanager.start_uid
    def __next__(self):
        element = self._listmanager.get_element(self._pos)
        if element is None:
            raise StopIteration()
        return element

class UnreachableElementsError(Exception):
    def __init__(self, elements):
        super().__init__('Unreachable elements: {}'.format(elements))
        self.elements = elements

class IncorrectLinkingError(Exception):
    def __init__(self, src, dest):
        super().__init__('Incorrect link betweek {src} and {dest}'
            .format(src=src, dest=dest))
        self.src = src
        self.dest = dest

class ListManager(_collections.abc.Iterable):
    def __init__(self):
        self._data = {}
        self.start_uid = 0
    def get_element(self, uid):
        if uid != 0:
            return self._data[uid]
        else:
            return None
    def __iter__(self):
        return LinkedListIterator(self)
    def _validate(self):
        visited = []
        for element in self:
            if visited:
                if element.prev != visited[-1].uid
                    raise IncorrectLinkingError(
                        src=visited[-1].uid,
                        dest=element.uid)
            if element in visited:
                break
            visited.append(element)
        diff = set(self._data) - {e.uid for e in visited}
        if diff:
            raise UnreachableElementsError(diff)
    def set_element(self, element):
        uid = element.uid
        assert uid not in self._data

        nxt = self.get_element(element.next)
        prev = self.get_element(element.prev)
        if nxt is not None:
            nxt = nxt._replace(prev=uid)
            self._data[nxt.uid] = nxt
        if prev is not None:
            prev = prev._replace(next=uid)
            self._data[prev.uid] = prev

        self._data[uid] = element
        self._validate()
    def delete_element(self, uid):
        element = self.get_element(uid)
        nxt = self.get_element(element.next)
        prev = self.get_element(element.prev)

        if nxt is prev:
            self._data[prev.uid] = prev._replace(
                next=0)
        else:
            self._data[nxt.uid] = nxt._replace(
                prev=prev.uid)
            self._data[prev.uid] = prev._replace(
                next=nxt.uid)
        self._validate()
