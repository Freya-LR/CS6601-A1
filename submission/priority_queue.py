#####################################################
# CS 6601 - Assignment 1͏︌͏󠄁͏︉
# priority_queue.py͏︌͏󠄁͏︉
#####################################################

# DO NOT ADD OR REMOVE ANY IMPORTS FROM THIS FILE͏︌͏󠄁͏︉
import math

# Credits if any͏︌͏󠄁͏︉
# 1)͏︌͏󠄁͏︉
# 2)͏︌͏󠄁͏︉
# 3)͏︌͏󠄁͏︉

class PriorityQueue(object):
    """
    A queue structure where each element is served in order of priority.
    Elements in the queue are popped based on the priority with higher priority
    elements being served before lower priority elements.  If two elements have
    the same priority, they will be served in the order they were added to the
    queue.
    Traditionally priority queues are implemented with heaps, but there are any
    number of implementation options.
    (Hint: take a look at the module heapq)
    You may add extra helper functions within the class if you find them necessary.
    Attributes:
        queue (list): Nodes added to the priority queue.

    Note:
        We have provided basic PQ implementation for you. You may find it helpful
        to tailor the implementation to your own usage (or to simply replace it with
        your implementation). This section will not be graded.
    """

    def __init__(self):
        """Initialize a new Priority Queue."""

        self.queue = []
        self.counter = 0 # FIFO Tracker

    def pop(self):
        """
        Pop top priority node from queue.
        Returns:
            The node with the highest priority.
        """
        
        # Obtain the top value͏︌͏󠄁͏︉
        top_value = self.top()

        # Re-heapify the queue͏︌͏󠄁͏︉
        # 1) Replace Top with Bottom͏︌͏󠄁͏︉
        self.queue[0] = self.queue[-1]

        # 2) Delete Bottom͏︌͏󠄁͏︉
        self.queue.pop()

        # 3) Bubble down until no more swapping occurs͏︌͏󠄁͏︉
        i = 0
        i_left = (i*2) + 1
        i_right = (i*2) + 2
        while (i_left < self.size()) or (i_right < self.size()):
            i_swap = i
            # locate the smallest node out of parent, left child, and right child͏︌͏󠄁͏︉
            if (i_left < self.size()) and self.compare(i_left, i_swap):
                i_swap = i_left
            if (i_right < self.size()) and self.compare(i_right, i_swap):
                i_swap = i_right
            # Perform the swap͏︌͏󠄁͏︉
            if (i_swap == i):
                # no swapping, we are done͏︌͏󠄁͏︉
                break
            else:
                self.swap(i, i_swap)
                i = i_swap
                i_left = (i*2) + 1
                i_right = (i*2) + 2
        
        return top_value

    def remove(self, node):
        """
        Remove a node from the queue.
        Hint: You might require this in ucs. However, you may
        choose not to use it or to define your own method.
        Args:
            node (tuple): The node to remove from the queue.
        """
        
        # We will not test this function, implementation and desired behavior is up to your discretion͏︌͏󠄁͏︉
        # Some students find that this function is useful for them in Assignment 1͏︌͏󠄁͏︉
        raise NotImplementedError

    def __iter__(self):
        """Queue iterator."""

        return iter(sorted(self.queue))

    def __str__(self):
        """Priority Queue to string."""

        return 'PQ:%s' % self.queue

    def append(self, node):
        """
        Append a node to the queue.
        Args:
            node (tuple): Comparable Object to be added to the priority queue.
            Provided in the form of (int priority, any type payload)
        """

        # Wrap the node in a counter͏︌͏󠄁͏︉
        wrapped_node = (self.counter, node)

        # Update the counter͏︌͏󠄁͏︉
        self.counter += 1

        # Append the node ot the end of the queue͏︌͏󠄁͏︉
        self.queue.append(wrapped_node)

        # Bubble up the node until no swapping occurs͏︌͏󠄁͏︉
        i = self.size() - 1
        i_parent = (i-1)//2
        while (i != 0) and self.compare(i, i_parent):
            self.swap(i, i_parent)
            i = i_parent
            i_parent = (i-1)//2

    def __contains__(self, key):
        """
        Containment Check operator for 'in'
        Args:
            key: The key to check for in the queue.
        Returns:
            True if key is found in queue, False otherwise.
        """

        return key in [n[-1] for n in self.queue]

    def __eq__(self, other):
        """
        Compare this Priority Queue with another Priority Queue.
        Args:
            other (PriorityQueue): Priority Queue to compare against.
        Returns:
            True if the two priority queues are equivalent.
        """

        return self.queue == other.queue

    def size(self):
        """
        Get the current size of the queue.
        Returns:
            Integer of number of items in queue.
        """

        return len(self.queue)

    def clear(self):
        """Reset queue to empty (no nodes)."""

        self.queue = []

    def top(self):
        """
        Get the top item in the queue.
        Returns:
            The first item stored in the queue.
        """

        return self.queue[0][1]
    
    def swap(self, a, b):
        """
        Swaps two nodes in the queue given their indices
        """

        tmp = self.queue[a]
        self.queue[a] = self.queue[b]
        self.queue[b] = tmp

    def compare(self, a, b):
        """
        Compare two nodes in queue given their indices
        
        Returns:
            True if queue[a].priority < queue[b].priority
            True if queue[a].priority == queue[b].priority and queue[a].FIFO < queue[b].FIFO
            False otherwise
            Means queue[a] has higher priority than queue[b]
        """

        # Compare priorities͏︌͏󠄁͏︉
        if (self.queue[a][1][0] < self.queue[b][1][0]):
            return True
        elif (self.queue[a][1][0] == self.queue[b][1][0]) and (self.queue[a][0] < self.queue[b][0]):
            return True
        else:
            return False