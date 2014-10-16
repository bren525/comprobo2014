"""priorityqueue.py --- a simple priority queue.

dyoo@hkn.eecs.berkeley.edu

A simple implementation of a priority queue.  It's not pretty, nor is
it that efficient.  Still, it's usable, and I know that I can improve
it by using an array heap later."""

class PriorityQueue:
    def __init__(self):
        self.values = []

    def __len__(self):
        """Return the number of elements in the queue."""
        return len(self.values)

    def push(self, priority, value):
        """Add an element to the queue.

        Only add if the value doesn't already exist in the queue, or
        if the priority is smaller than the one existing in the queue.

        If addition is successful, then return 1.  Else, return 0."""
        search_result = self._find(value)
        if search_result and search_result[0] <= priority:
            return 0
        elif search_result:
            del self.values[self.values.index(search_result)]
        self.values.append((priority, value))
        self.values.sort(_psort)
        return 1

    def pop(self):
        """Return the element with the smallest priority."""
        assert not self.isEmpty()
        priority, value = self.values[0]
        del self.values[0]
        return (priority, value)

    def isEmpty(self):
        """return 1 if the queue is empty, 0 otherwise."""
        return len(self.values) == 0

    def _find(self, value):
        """Helper function to see if a value already exists in the queue."""
        for (p, v) in self.values:
            if value == v: return (p, v)
        return None

def _psort(x, y):
    return cmp(x[0], y[0])

def _test():
    """We expect to see:
(1, 'd')
(5, 'a')
(12, 'b')
(14, 'c')
    """
    x = PriorityQueue()
    data = [(10, 'a'), (12, 'b'), (14, 'c'), (5, 'a'), (1, 'd')]
    for p, v in data:
        x.push(p, v)
    while not x.isEmpty():
        print x.pop()

if __name__ == '__main__': _test()