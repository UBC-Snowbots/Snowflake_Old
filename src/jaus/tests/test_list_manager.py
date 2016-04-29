import pytest

from jaus.list_manager import(

    ListManager,
    ListElement
)

#Test that inserting an element into an empty List works as intended
def test_first_insertion():

    list_manager = ListManager()

    new_element = ListElement(5,0,0)

    list_manager.setElement(new_element)

    assert(new_element in list_manager.getElementSet())


#Tests the following case: Add a head element, add a tail element, then add an element in between the head and tail
def test_multiple_insertions():

    list_manager = ListManager()

    head_element = ListElement(1,0,0)
    tail_element = ListElement(2,1,0)
    middle_element = ListElement(3,1,2)

    list_manager.setElement(head_element)
    list_manager.setElement(tail_element)
    list_manager.setElement(middle_element)

    assert(head_element in list_manager.getElementSet())
    assert(tail_element in list_manager.getElementSet())
    assert(middle_element in list_manager.getElementSet())
    
    list_head = list_manager.getHead()
    assert(list_head == head_element)

    list_second = list_manager.getElement(list_head.nextUID)
    assert(list_second == middle_element)

    list_third = list_manager.getElement(list_second.nextUID)
    assert(list_third == tail_element)

    assert(list_third.nextUID == 0)

def test_head_element_deletion():

    list_manager = ListManager()

    head_element = ListElement(1,0,0)
    tail_element = ListElement(2,1,0)
    middle_element = ListElement(3,1,2)

    list_manager.setElement(head_element)
    list_manager.setElement(tail_element)
    list_manager.setElement(middle_element)

    list_manager.deleteElement(head_element)

    assert(len(list_manager.getElementSet()) == 2)

    assert(list_manager.getHead() == middle_element)
    
    list_second = list_manager.getElement(list_manager.getHead().nextUID)
    assert(list_second == tail_element)

    assert(list_second.nextUID == 0)

def test_tail_element_deletion():
    list_manager = ListManager()

    head_element = ListElement(1,0,0)
    tail_element = ListElement(2,1,0)
    middle_element = ListElement(3,1,2)

    list_manager.setElement(head_element)
    list_manager.setElement(tail_element)
    list_manager.setElement(middle_element)

    list_manager.deleteElement(tail_element)

    assert(len(list_manager.getElementSet()) == 2)

    assert(list_manager.getHead() == head_element)
    
    list_second = list_manager.getElement(list_manager.getHead().nextUID)
    assert(list_second == middle_element)

    assert(list_second.nextUID == 0)



def test_middle_element_deletion():
    list_manager = ListManager()

    head_element = ListElement(1,0,0)
    tail_element = ListElement(2,1,0)
    middle_element = ListElement(3,1,2)

    list_manager.setElement(head_element)
    list_manager.setElement(tail_element)
    list_manager.setElement(middle_element)

    list_manager.deleteElement(middle_element)

    assert(len(list_manager.getElementSet()) == 2)

    assert(list_manager.getHead() == head_element)
    
    list_second = list_manager.getElement(list_manager.getHead().nextUID)
    assert(list_second == tail_element)

    assert(list_second.nextUID == 0)

def test_complete_deletion():
    list_manager = ListManager()

    head_element = ListElement(1,0,0)
    tail_element = ListElement(2,1,0)
    middle_element = ListElement(3,1,2)

    list_manager.setElement(head_element)
    list_manager.setElement(tail_element)
    list_manager.setElement(middle_element)

    list_manager.deleteElement(middle_element)
    list_manager.deleteElement(head_element)

    assert(len(list_manager.getElementSet()) == 1)

    assert(list_manager.getHead() == tail_element)
    list_manager.deleteElement(tail_element)

    assert(len(list_manager.getElementSet()) == 0)
   
def test_circular_list():
    list_manager = ListManager()

    first_element = ListElement(1,0,0)
    second_element = ListElement(2,1,0)
    third_element = ListElement(3,2,0)
    fourth_element = ListElement(4,3,2)

    list_manager.setElement(first_element)
    list_manager.setElement(second_element)
    list_manager.setElement(third_element)
    list_manager.setElement(fourth_element)



    fifth_element = ListElement(5,4,2)
    list_manager.setElement(fifth_element)

    list_head = list_manager.getHead()
    assert(list_head == first_element)

    list_second = list_manager.getElement(list_head.nextUID)
    assert(list_second == second_element)

    assert list_second.isLoopStart()

    list_third = list_manager.getElement(list_second.nextUID)
    assert(list_third == third_element)

    list_fourth = list_manager.getElement(list_third.nextUID)
    assert(list_fourth == fourth_element)
    
    list_fifth = list_manager.getElement(list_fourth.nextUID)
    assert(list_fifth == fifth_element)

    assert( list_manager.getElement(list_fifth.nextUID) == list_second)
  
def test_circular_deletion():
    list_manager = ListManager()

    first_element = ListElement(1,0,0)
    second_element = ListElement(2,1,0)
    third_element = ListElement(3,2,0)
    fourth_element = ListElement(4,3,2)

    list_manager.setElement(first_element)
    list_manager.setElement(second_element)
    list_manager.setElement(third_element)
    list_manager.setElement(fourth_element)
    

    list_manager.deleteElement(second_element)

    list_head = list_manager.getHead()
    assert(list_head == first_element)

    list_second = list_manager.getElement(list_head.nextUID)
    assert(list_second == third_element)

    list_third = list_manager.getElement(list_second.nextUID)
    assert(list_third == fourth_element)

    list_fourth = list_manager.getElement(list_third.nextUID)
    assert(list_fourth == third_element)


#Tests that elements which point to themselves are rejected

def test_self_circular():

    list_manager = ListManager()

    first_element = ListElement(1,1,1)
    
    assert(len(list_manager.getElementSet()) == 0)


    first_element = ListElement(1,0,0)
    second_element = ListElement(2,1,0)
    third_element = ListElement(3,2,0)
    fourth_element = ListElement(4,3,2)

    list_manager.setElement(first_element)
    list_manager.setElement(second_element)
    list_manager.setElement(third_element)
    list_manager.setElement(fourth_element)

    list_manager.deleteElement(second_element)
    list_manager.deleteElement(third_element) #Expect this deletion to fail

    list_head = list_manager.getHead()
    assert(list_head == first_element)

    list_second = list_manager.getElement(list_head.nextUID)
    assert(list_second == third_element)

    list_third = list_manager.getElement(list_second.nextUID)
    assert(list_third == fourth_element)

    list_fourth = list_manager.getElement(list_third.nextUID)
    assert(list_fourth == third_element)
    
""""
Tests multiple requests that should be rejected, such as:

-Inserting an element which already exists


The list will begin (and should end) in the following form:
    17->83->21->4->18->26->21 (circular)
"""
def test_multiple_rejections():
    
    list_manager = ListManager()

    first_element = ListElement(17,0,0)
    second_element = ListElement(83,17,0)
    third_element = ListElement(21,83,0)
    fourth_element = ListElement(4,21,0)
    fifth_element = ListElement(18,4,0)
    sixth_element = ListElement(26,18,21)

    list_manager.setElement(first_element)
    list_manager.setElement(second_element)
    list_manager.setElement(third_element)
    list_manager.setElement(fourth_element)
    list_manager.setElement(fifth_element)
    list_manager.setElement(sixth_element)



    #Now it's time for the requests which need to be rejected

    repeatedElement = ListElement(83,4,18)
    list_manager.setElement(repeatedElement) #Add element with an ID that already exists


    secondLoopElement = ListElement(211,4,21)
    list_manager.setElement(secondLoopElement) #Add a second loop



    list_head = list_manager.getHead()
    assert(list_head == first_element)

    list_second = list_manager.getElement(list_head.nextUID)
    assert(list_second == second_element)

    list_third = list_manager.getElement(list_second.nextUID)
    assert(list_third == third_element)

    list_fourth = list_manager.getElement(list_third.nextUID)
    assert(list_fourth == fourth_element)

    list_fifth = list_manager.getElement(list_fourth.nextUID)
    assert(list_fifth == fifth_element)

    list_sixth = list_manager.getElement(list_fifth.nextUID)
    assert(list_sixth == sixth_element)

    list_seventh = list_manager.getElement(list_sixth.nextUID)
    assert(list_seventh == third_element)