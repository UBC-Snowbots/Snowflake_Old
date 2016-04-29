
"""
The list manager is a doubly linked list that can contain a cycle, thus there is a possibility of one node having 
two previous nodes

The manager contains the following functions:

setElement: Adds an element to the list, as long as it is a valid request

deleteElement: Removes an element from the list, as long as the request is valid

getElementSet: Returns a set of all elements in the list

getHead: Returns the head of the list, the node which has a previous node with UID of 0

getElement: Takes in a UID and returns an element with that UID


I don't know exactly how to return a 'RejectElementRequest' so that will need to be implemented
later, for now I just added a comment

Known errors:
Deleting an element in 1->2->3->2 does not work if removing the element 3, however
trying to remove 2 will fail as intended
"""
class ListManager:
    def __init__(self):
        self.element_set = set()


    def setElement(self, element):
        
        #Do not allow elements with multiple previous nodes to be set, or element with UID 0
        if(len(element.previousList) > 1 or element.UID == 0):
            """Return a RejectElementRequest here"""
            return

        #"""Case 1 : Element is being added to an empty list"""
        if(len(self.element_set) == 0):
            if(element.isPrevious(0) and element.nextUID == 0):
               self.element_set.add(element)
               return

        #This is a check for if the element already exists in the list
        for e in self.element_set:
            if (e.UID == element.UID):
                """Return a RejectElementRequest here"""
                return

        #"""Case 2: Element is being added to the beginning of the list"""
        if(element.isPrevious(0)):
            for e in self.element_set:
                if(e.isPrevious(0) and element.nextUID == e.UID()): 
                    e.setPrevious(element.UID)
                    self.element_set.add(element)
                    break

        #"""Case 3: Element is being added to the end of the list"""
        elif(element.nextUID == 0):
            for e in self.element_set:
                if(e.nextUID == 0 and element.isPrevious(e.UID)):
                    e.setNext(element.UID)
                    self.element_set.add(element)
                    break

        #"""Case 4: Element is being added between two existing nodes"""
        else:
            previousElement = None
            nextElement = None
           
            for e in self.element_set:
                if(element.isPrevious(e.UID)):
                    previousElement = e
                if(element.nextUID == e.UID):
                    nextElement = e

            if(previousElement != None and nextElement != None):
                
                #Case 4.1: Element is not creating a circular loop
                if(nextElement.UID == previousElement.nextUID): 
                    previousElement.setNext(element.UID)
                    nextElement.addPrevious(element.UID)
                    nextElement.removePrevious(previousElement.UID)
                    
                    self.element_set.add(element)

                #Case 4.2: Element is creating a circular loop
                elif(previousElement.nextUID == 0):
                    previousElement.setNext(element.UID)
                    nextElement.addPrevious(element.UID)
                    self.element_set.add(element)

    def deleteElement(self, element):

        #"""Case 1: Element is not in the list"""
        if element not in self.element_set:
            """Return a "RejectElementRequest" message here"""
            return 

        #"""Case 2: There is 1 element in the list"""
        if(len(self.element_set) == 1):
            self.element_set.remove(element)
            return

        nextElement = None
        previousElement = None

        #"""Case 3: Element to be removed is the head of the list"""
        if(element.isPrevious(0)):
            for e in self.element_set:
                if(element.nextUID == e.UID):
                    nextElement = e

            if(nextElement == None):
                print("The code really should never have reached this point...")
                """Return a "RejectElementRequest" here"""
                return

            nextElement.removePrevious(element.UID)
            self.element_set.remove(element)

        #"""Case 4: Element to be removed is at the end of the list"""

        elif(element.nextUID == 0):
            for e in self.element_set:
                if(element.isPrevious(e.UID)):
                    previousElement = e

            if(previousElement == None):
                print("The code really should never have reached this point...")
                """Return a "RejectElementRequest" here"""
                return

            previousElement.setNext(0)
            self.element_set.remove(element)

        #"""Case 5: Element to be removed is somewhere in the middle of the list"""
        else:
            previousList = []
            nextElement = None

            for e in self.element_set:
                if(element.isPrevious(e.UID)):
                    previousList.append(e)
                if(element.nextUID == e.UID):
                    nextElement = e
            
            
            if(nextElement in previousList):
                """retun RejectElementRequest here"""
                return

            nextElement.clearPreviousList()
            for p in previousList:
                p.setNext(element.nextUID)
                nextElement.addPrevious(p.UID)

           
            self.element_set.remove(element)

    def getElementSet(self):
        return self.element_set

    def getHead(self):
        for e in self.element_set:
            if(e.isPrevious(0)):
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
        self.previousList = [previousUID]
        self.UID = UID
        self.nextUID = nextUID
        
    def setNext(self, next):
        self.nextUID = next

    def setPrevious(self, previous):
        self.previousList = [previous]

    def setPreviousList(self, list):
        self.previousList = list

    def addPrevious(self, previous):
        self.previousList.append(previous)
        if(0 in self.previousList):
            previousList.remove(0)

    def clearPreviousList(self):
        self.previousList = []

    def isLoopStart(self):
        return (len(self.previousList) == 2)

    def isPrevious(self, element):
        return (element in self.previousList)

    def removePrevious(self, id):
        self.previousList.remove(id)
        if(len(self.previousList) == 0):
            self.previousList.append(0)
    
