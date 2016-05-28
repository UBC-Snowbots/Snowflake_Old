
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
class ListManager:
    def __init__(self):
        self.element_set = set()


    def setElement(self, element):
        
        #Do not allow elements with UID 0
        if(element.UID == 0):
            raise RejectElementRequest("Element has a UID of 0")
            return

        #"""Case 1 : Element is being added to an empty list"""
        if(len(self.element_set) == 0):
            if(element.previousUID == 0 and element.nextUID == 0):
               self.element_set.add(element)
               return

        #This is a check for if the element already exists in the list
        for e in self.element_set:
            if (e.UID == element.UID):
                raise RejectElementRequest("Element already exists in the list")
                return

        #"""Case 2: Element is being added to the beginning of the list"""
        if(element.previousUID == 0):
            for e in self.element_set:
                if(e.previousUID == element.UID and element.nextUID == e.UID()): 
                    e.setPrevious(element.UID)
                    self.element_set.add(element)
                    return

        #"""Case 3: Element is being added to the end of the list"""
        elif(element.nextUID == 0):
            for e in self.element_set:
                if(e.nextUID == 0 and element.previousUID == e.UID):
                    e.setNext(element.UID)
                    self.element_set.add(element)
                    return

        #"""Case 4: Element is being added between two existing nodes"""
        else:
            previousElement = None
            nextElement = None
           
            for e in self.element_set:
                if(element.previousUID == e.UID):
                    previousElement = e
                if(element.nextUID == e.UID):
                    nextElement = e

            if(previousElement != None and nextElement != None):
                if(nextElement.UID == previousElement.nextUID or previousElement.nextUID == 0): 
                    previousElement.setNext(element.UID)
                    nextElement.setPrevious(element.UID)
                    self.element_set.add(element)
                    return

        raise RejectElementRequest("Element cannot be added to the list") ##if any of the cases failed

    def deleteElement(self, element):

        #Case where element is not in the list
        if element not in self.element_set:
            raise RejectElementRequest("Element is not in the list")
            return 

        #Check if there is only 1 element in the list
        if(len(self.element_set) == 1):
            self.element_set.remove(element)
            return

        nextElement = None
        previousElement = None
        isLoopStart = False;
        for e in self.element_set:
            if(element.nextUID == e.UID):
                nextElement = e
            if(element.previousUID == e.UID):
                previousElement = e
            if(e.nextUID == element and element.previousUID != e.UID):
                isLoopStart = True
        
        if(isLoopStart):
            raise RejectElementRequest("Element is the start of a loop, cannot be removed")
            return
        if(previousElement == None):
            nextElement.setPrevious(0)
            self.element_set.remove(element)
            return
        elif(nextElement == None):
            previousElement.setNext(0)
            self.element_set.remove(element)
            return
        elif(nextElement == previousElement):
            previousElement.nextUID = 0
            self.element_set.remove(element)
            return
        else:
            previousElement.setNext(element.nextUID)
            nextElement.setPrevious(previousElement.UID)
            self.element_set.remove(element)
            return

        raise RejectElementRequest("Element could not be removed")

    def getElementSet(self):
        return self.element_set

    def getHead(self):
        for e in self.element_set:
            if(e.previousUID == 0):
                return e
        return None  
    
    def getElement(self, UID):
        for e in self.element_set:
            if(e.UID == UID):
                return e
        return None
         
class ListElement:
    """
    A list is used for previous elements since, in a circular list, 
    a node can have more than one previous element
    """
    def __init__(self, UID, previousUID, nextUID):
        self.previousUID = previousUID
        self.UID = UID
        self.nextUID = nextUID
        
    def setNext(self, next):
        if(next == self.UID):
            self.nextUID = 0
        else: 
            self.nextUID = next

    def setPrevious(self, previous):
        if(previous == self.UID):
            self.previousUID = 0
        else:
            self.previousUID = previous

 
class RejectElementRequest(Exception):
    pass
