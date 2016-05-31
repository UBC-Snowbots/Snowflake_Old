import collections.abc as _abc
import logging as _logging

"""
Use python3 -m pytest to test


The manager contains the following functions:

setElement: Adds an element to the list, as long as it is a valid request

deleteElement: Removes an element from the list, as long as the request is valid

getElementSet: Returns a set of all elements in the list

getHead: Returns the head of the list, the node which has a previous node with UID of 0

getElement: Takes in a UID and returns an element with that UID

If a node is the start of a loop (two nodes point to it), it will reject a delete request
"""
class Element:
    def __init__(self, uid, next, prev, data):
        self.uid = uid
        self.next = next
        self.prev = prev
        self.data = data
    def __repr__(self):
        return "Element(uid={uid}, next={next}, prev={prev}, data={data})".format(**self.__dict__)
    def copy(self):
        return Element(uid=self.uid, next=self.next, prev=self.prev, data=self.data)
    def __eq__(self, other):
        return (
            other.uid == self.uid
            and other.next == self.next
            and other.prev == self.prev
            and other.data == self.data)

def to_list(lm):
    visited = set()
    order = []
    for e in lm:
        if e.uid in visited:
            return order
        visited |= {e.uid}
        order.append(e)
    return order

class ListError(Exception):
    pass

class NoSuchElement(ListError):
    def __init__(self, element, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.element = element
    def __str__(self):
        return "No such element: {}".format(self.element)

class ElementAlreadyExists(ListError):
    def __init__(self, element, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.element = element
    def __str__(self):
        return "Element already exists: {}".format(self.element)

class IncorrectHeadCount(ListError):
    def __init__(self, head_count, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.head_count = head_count
    def __str__(self):
        return "Incorrect head count: {}".format(self.head_count)

class IncorrectTailCount(ListError):
    def __init__(self, tail_count, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.tail_count = tail_count
    def __str__(self):
        return "Incorrect tail count: {}".format(self.tail_count)

class BrokenReference(ListError):
    def __init__(self, element, uid, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.element = element
        self.uid = uid
    def __str__(self):
        return "Broken reference in {} to {}".format(self.element, self.uid)

class OrphanedElements(ListError):
    def __init__(self, elements, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.elements = elements
    def __str__(self):
        return "Orphaned elements: {}".format(self.elements)

class SelfReference(ListError):
    def __init__(self, elements, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.element = element
    def __str__(self):
        return "Self-reference on element: {}".format(self.element)

class Iterator(_abc.Iterator):
    def __init__(self, list_manager):
        self._list_manager = list_manager
        head = list_manager.head
        self._current_uid = head.uid if head else None
    def __next__(self):
        if self._current_uid == 0 or self._current_uid is None:
            raise StopIteration()
        element = self._list_manager[self._current_uid]
        self._current_uid = element.next
        return element

class ListManager(_abc.MutableMapping):
    def __init__(self):
        self._elements = {}

    def __getitem__(self, uid):
        return self._elements[uid]

    def __setitem__(self, uid, element):
        if uid != element.uid:
            raise ValueError()
        self.insert_batch([element])

    def insert_batch(self, elements):
        old_elements = self._elements
        new_elements = self._copy_elements()
        for e in elements:
            if e.uid in new_elements:
                raise ElementAlreadyExists(e)
            uid = e.uid
            new_elements[uid] = e
            replaces_head = False
            if e.prev in old_elements:
                prev = new_elements.get(e.prev)
                if prev:
                    if prev.next == 0:
                        replaces_head = True
                    prev.next = e.uid
            if e.next in old_elements and not replaces_head:
                next = new_elements.get(e.next)
                if next:
                    next.prev = e.uid

        self._check_validity(new_elements)
        self._elements = new_elements

    def delete_batch(self, uids):
        new_elements = self._copy_elements()
        tail = self._find_tail(new_elements)
        for uid in uids:
            if uid not in new_elements:
                raise NoSuchElement(uid)
            e = new_elements[uid]
            next = new_elements.get(e.next)
            prev = new_elements.get(e.prev)
            # the tail checks are there to make it possible to break cycles
            # with deletes
            if prev is not None:
                if uid == tail.uid:
                    prev.next = 0
                else:
                    prev.next = e.next
            if next is not None:
                if uid != tail.uid:
                    next.prev = e.prev
            del new_elements[uid]
        _logging.debug('pre-delete: {}; post-delete: {}'.format(
            self._elements, new_elements))
        self._check_validity(new_elements)
        self._elements = new_elements

    @classmethod
    def _check_validity(self, elements):
        uids = elements.keys()
        ## empty list is always ok
        if elements == {}:
            return

        # test unique head
        head_count = sum(1 for e in elements.values() if e.prev == 0)
        if head_count != 1:
            raise IncorrectHeadCount(head_count)

        # either circular or has one tail
        tail_count = sum(1 for e in elements.values() if e.next == 0)
        if tail_count > 1:
            raise IncorrectTailCount(tail_count)

        # all elements must reference existing elements
        for e in elements.values():
            next = e.next
            prev = e.prev
            if next != 0 and next not in uids:
                raise BrokenReference(e, next)
            if prev != 0 and prev not in uids:
                raise BrokenReference(e, prev)

        # no elements must be orphaned
        # find head
        tail = [e for e in elements.values() if e.prev == 0][0]
        visited_elements = {tail.uid}
        while tail.next != 0 and tail.next not in visited_elements:
            tail = elements[tail.next]
            visited_elements |= {tail.uid}
        # make sure the tail and tail count are consistent
        if tail_count == 1:
            if tail.next != 0:
                raise IncorrectTailCount(1)
        else:
            if tail.next == 0:
                raise IncorrectTailCount(0)
        if visited_elements != uids:
            # we have orphaned elements
            raise OrphanedElements(uids - visited_elements)

        # not self-references
        for e in elements.values():
            if e.next == e.uid or e.prev == e.uid:
                # self-reference detected
                raise SelfReference(e)

    def _copy_elements(self):
        return {k: v.copy() for k, v in self._elements.items()}

    def __delitem__(self, uid):
        self.delete_batch([uid])

    def __len__(self):
        return len(self._elements)

    @property
    def head(self):
        for e in self._elements.values():
            if e.prev == 0:
                return e
        return None

    @classmethod
    def _find_tail(self, elements):
        visited = set()
        for e in elements.values():
            if e.next == 0:
                return e
            elif e.next in visited:
                return e
            visited |= {e.uid}
        return None

    @property
    def tail(self):
        return self._find_tail(self._elements)

    def __iter__(self):
        return Iterator(self)

    @property
    def count(self):
        return len(self._elements)
