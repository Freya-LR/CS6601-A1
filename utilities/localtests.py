# coding=utf-8͏︌͏󠄁͏︉

# Author:͏︌͏󠄁͏︉
# Last Updated: 01/11/2025 by Raymond Jia͏︌͏󠄁͏︉

import pickle
import unittest
import networkx

import random

from helpers.explorable_graph import ExplorableGraph

def print_success_message(msg):
    print(f'UnitTest passed successfully for "{msg}"!')

# Class for Priority Queue testing͏︌͏󠄁͏︉
class TestPriorityQueue(unittest.TestCase):
    """Test Priority Queue implementation"""
    def __init__(self, methodName: str = ...) -> None:
        super().__init__(methodName)

    def test_append_and_pop(self, PriorityQueue):
        """Test the append and pop functions"""
        queue = PriorityQueue()
        temp_list = []

        for _ in range(10):
            a = random.randint(0, 10000)
            queue.append((a, 'a'))
            temp_list.append(a)

        temp_list = sorted(temp_list)

        for item in temp_list:
            popped = queue.pop()
            self.assertEqual(popped[0], item)
        
        print_success_message(self._testMethodName)

    def test_fifo_property(self, PriorityQueue):
        "Test the fifo property for nodes with same priority"
        queue = PriorityQueue()
        temp_list = [(1, 'b'), (1, 'c'), (1, 'a')]

        for node in temp_list:
            queue.append(node)
        
        for expected_node in temp_list:
            actual_node = queue.pop()
            self.assertEqual(actual_node[-1], expected_node[-1])

        print_success_message(self._testMethodName)

# Class for BFS testing͏︌͏󠄁͏︉
class TestBFS(unittest.TestCase):
    """Test BFS Implementation"""
    def __init__(self, methodName: str = ...) -> None:
        super().__init__(methodName)

        # Setup graph͏︌͏󠄁͏︉
        with(open("romania/romania_graph.pickle", "rb")) as romania_file:
            romania = pickle.load(romania_file)
        self.romania = ExplorableGraph(romania)
        self.romania.reset_search()

        # Setup references͏︌͏󠄁͏︉
        with(open("romania/romania_references.pickle", "rb")) as romania_ref_file:
            self.romania_refs = pickle.load(romania_ref_file)
    
    @staticmethod
    def is_continuous_path(graph, path):
        """
        Checks if the provided path is a continuous path

        Args:
            graph (ExplorableGraph): Undirected graph with path
            path (list(nodes)): List of nodes

        Returns:
            Boolean True if path is continuous, False otherwise
        """

        for i in range(0, len(path)-1):
            edges = networkx.edges(graph, path[i])
            if not any([path[i+1] == v for e,v in edges]):
                return False
        return True
    
    def is_valid_path(self, graph, src_node, dst_node, path):
        """
        Checks if the provided path (two goals) is a valid path

        Args:
            graph (ExplorableGraph): Undirected graph with path
            src_node (node): Key for the start node
            dst_node (node): Key for the end node
            path (list(nodes)): List of nodes from src to dst

        Returns:
            Boolean True if path is valid, False otherwise
        """

        # Check if stationary path͏︌͏󠄁͏︉
        if src_node == dst_node:
            return not path
        
        # Check endpoints͏︌͏󠄁͏︉
        if not ((path[0] == src_node and path[-1] == dst_node) or (path[-1] == src_node and path[0] == dst_node)):
            return False
        
        # Check path continuity͏︌͏󠄁͏︉
        return self.is_continuous_path(graph, path)
    
    def is_optimal_path(self, case, path):
        """
        Check if path provided is shortest path matching references

        Args:
            case (tuple(node)): Goal nodes
            path (list(node)): Student's solution path
        
        Returns:
            Boolean, Path Cost, Expected Cost
        """
        
        path_cost = len(path)
        ref_path = self.romania_refs["test_bfs_romania"][case][0]
        # Check if ref_path is accidentally flipped͏︌͏󠄁͏︉
        if len(path) > 0 and ref_path[-1] == path[0]:
            ref_path = ref_path[::-1]
        ref_path_cost = len(ref_path)

        matches_ref = path == ref_path
        return matches_ref, ref_path, path_cost, ref_path_cost
    
    def is_allowed_explored_cnt(self, case, explored):
        """
        Check if explored is within allowed bounds of reference

        Args:
            case (tuple(node)): Goal nodes
            path (dict(node: int)): Student's explored

        Returns:
            Boolean, Explored Count, Expected Explored Count Max
        """

        explored_cnt = sum(explored.values())
        ref_explored_cnt_max = self.romania_refs["test_bfs_romania"][case][1]

        is_within_bounds = explored_cnt <= ref_explored_cnt_max
        return is_within_bounds, explored_cnt, ref_explored_cnt_max
    
    def test_valid_paths(self, breadth_first_search):
        # Collect all student paths͏︌͏󠄁͏︉
        student_solns = {}
        for goal_case in self.romania_refs["test_bfs_romania"]:
            self.romania.reset_search()
            student_solns[goal_case] = (breadth_first_search(self.romania, goal_case[0], goal_case[1]), self.romania.explored_nodes())

        # Check if all paths are valid͏︌͏󠄁͏︉
        failed = 0
        sample_failed_case = ""
        for case in student_solns:
            verdict = self.is_valid_path(self.romania, case[0], case[1], student_solns[case][0])
            if not verdict:
                failed += 1
                if failed == 1:
                    sample_failed_case = "Path %s for start node '%s' to goal node '%s' is not a valid path" % (student_solns[case][0], case[0], case[1])
        self.assertTrue(failed == 0, msg="Not all paths found are valid!\nYou failed " + str(failed) + " cases out of " + str(len(student_solns)) + " cases.\nAn example failed case:\n" + sample_failed_case)
        
        print_success_message(self._testMethodName)

    def test_optimal_paths(self, breadth_first_search):
        # Collect all student paths͏︌͏󠄁͏︉
        student_solns = {}
        for goal_case in self.romania_refs["test_bfs_romania"]:
            self.romania.reset_search()
            student_solns[goal_case] = (breadth_first_search(self.romania, goal_case[0], goal_case[1]), self.romania.explored_nodes())

        # Check if all paths are optimal͏︌͏󠄁͏︉
        failed = 0
        sample_failed_case = ""
        for case in student_solns:
            verdict, expected_path, path_cost, expected_path_cost = self.is_optimal_path(case, student_solns[case][0])
            if not verdict:
                failed += 1
                if failed == 1:
                    sample_failed_case = "Path %s for goal nodes %s does not match reference. Path cost was %s and expected path cost was %s. Expected path is %s" % (student_solns[case][0], case, path_cost, expected_path_cost, expected_path)
        self.assertTrue(failed == 0, msg="Not all paths found were optimal!\nYou failed " + str(failed) + " cases out of " + str(len(student_solns)) + " cases.\nAn example failed case:\n" + sample_failed_case)
        
        print_success_message(self._testMethodName)

    def test_explored_counts(self, breadth_first_search):
        # Collect all student paths͏︌͏󠄁͏︉
        student_solns = {}
        for goal_case in self.romania_refs["test_bfs_romania"]:
            self.romania.reset_search()
            student_solns[goal_case] = (breadth_first_search(self.romania, goal_case[0], goal_case[1]), self.romania.explored_nodes())

        # Check if all paths are optimal͏︌͏󠄁͏︉
        failed = 0
        sample_failed_case = ""
        for case in student_solns:
            verdict, explored_cnt, expected_explored_cnt_max = self.is_allowed_explored_cnt(case, student_solns[case][1])
            if not verdict:
                failed += 1
                if failed == 1:
                    sample_failed_case = "Path %s for goal nodes %s explored more nodes than allowed maximum. Explored count was %s and max allowed count was %s" % (student_solns[case][0], case, explored_cnt, expected_explored_cnt_max)
        self.assertTrue(failed == 0, msg="Explored counts exceeded for some paths!\nYou failed " + str(failed) + " cases out of " + str(len(student_solns)) + " cases.\nAn example failed case:\n" + sample_failed_case)
        
        print_success_message(self._testMethodName)

    def test_alternate_representation(self, breadth_first_search):
        # Setup graph͏︌͏󠄁͏︉
        with(open("romania/romania_graph2.pickle", "rb")) as romania_file:
            romania = pickle.load(romania_file)
        self.romania = ExplorableGraph(romania)

        # Setup references͏︌͏󠄁͏︉
        with(open("romania/romania_references2.pickle", "rb")) as romania_ref_file:
            self.romania_refs = pickle.load(romania_ref_file)
        
        self.romania.reset_search()
        self.test_valid_paths(breadth_first_search)
        self.romania.reset_search()
        self.test_optimal_paths(breadth_first_search)
        self.romania.reset_search()
        self.test_explored_counts(breadth_first_search)


# Class for two-goal search algorithm testing͏︌͏󠄁͏︉
class TestSearchAlgorithms(unittest.TestCase):
    """Test standard search algorithm implementations"""
    def __init__(self, methodName: str = ...) -> None:
        super().__init__(methodName)

        # Setup graph͏︌͏󠄁͏︉
        with(open("romania/romania_graph.pickle", "rb")) as romania_file:
            romania = pickle.load(romania_file)
        self.romania = ExplorableGraph(romania)
        self.romania.reset_search()

        # Setup references͏︌͏󠄁͏︉
        with(open("romania/romania_references.pickle", "rb")) as romania_ref_file:
            self.romania_refs = pickle.load(romania_ref_file)

    @staticmethod
    def get_path_cost(graph, path):
        """
        Calculate the total cost of a path by summing edge weights

        Args:
            graph (ExplorableGraph): Graph that contains the path
            path (list(nodes)): List of nodes from src to dst

        Returns:
            Sum of edge weights in path
        """

        pairs = zip(path, path[1:])
        return sum([graph.get_edge_data(a, b)['weight'] for a, b in pairs])
    
    @staticmethod
    def is_continuous_path(graph, path):
        """
        Checks if the provided path is a continuous path

        Args:
            graph (ExplorableGraph): Undirected graph with path
            path (list(nodes)): List of nodes

        Returns:
            Boolean True if path is continuous, False otherwise
        """

        for i in range(0, len(path)-1):
            edges = networkx.edges(graph, path[i])
            if not any([path[i+1] == v for e,v in edges]):
                return False
        return True
    
    def is_valid_path(self, graph, src_node, dst_node, path):
        """
        Checks if the provided path (two goals) is a valid path

        Args:
            graph (ExplorableGraph): Undirected graph with path
            src_node (node): Key for the start node
            dst_node (node): Key for the end node
            path (list(nodes)): List of nodes from src to dst

        Returns:
            Boolean True if path is valid, False otherwise
        """

        # Check if stationary path͏︌͏󠄁͏︉
        if src_node == dst_node:
            return not path
        
        # Check endpoints͏︌͏󠄁͏︉
        if not ((path[0] == src_node and path[-1] == dst_node) or (path[-1] == src_node and path[0] == dst_node)):
            return False
        
        # Check path continuity͏︌͏󠄁͏︉
        return self.is_continuous_path(graph, path)
    
    def is_optimal_path(self, test_name, case, path):
        """
        Check if path provided is shortest path matching references

        Args:
            test_name: Name of test
            case (tuple(node)): Goal nodes
            path (list(node)): Student's solution path
        
        Returns:
            Boolean, Path Cost, Expected Cost
        """
        
        path_cost = self.get_path_cost(self.romania, path)
        ref_path = self.romania_refs[test_name][case][0]
        # Check if ref_path is accidentally flipped͏︌͏󠄁͏︉
        if len(path) > 0 and ref_path[-1] == path[0]:
            ref_path = ref_path[::-1]
        ref_path_cost = self.get_path_cost(self.romania, ref_path)

        matches_ref = path == ref_path
        return matches_ref, ref_path, path_cost, ref_path_cost
    
    def is_allowed_explored_cnt(self, test_name, case, explored):
        """
        Check if explored is within allowed bounds of reference

        Args:
            test_name: Name of test
            case (tuple(node)): Goal nodes
            path (dict(node: int)): Student's explored

        Returns:
            Boolean, Explored Count, Expected Explored Count Max
        """

        explored_cnt = sum(explored.values())
        ref_explored_cnt_max = self.romania_refs[test_name][case][1]

        is_within_bounds = explored_cnt <= ref_explored_cnt_max
        return is_within_bounds, explored_cnt, ref_explored_cnt_max
    
    def test_valid_paths(self, test_name, test_func, **kwargs):
        # Collect all student paths͏︌͏󠄁͏︉
        student_solns = {}
        for goal_case in self.romania_refs[test_name]:
            self.romania.reset_search()
            student_solns[goal_case] = (test_func(self.romania, goal_case[0], goal_case[1], **kwargs), self.romania.explored_nodes())

        # Check if all paths are valid͏︌͏󠄁͏︉
        failed = 0
        sample_failed_case = ""
        for case in student_solns:
            verdict = self.is_valid_path(self.romania, case[0], case[1], student_solns[case][0])
            if not verdict:
                failed += 1
                if failed == 1:
                    sample_failed_case = "Path %s for start node '%s' to goal node '%s' is not a valid path" % (student_solns[case][0], case[0], case[1])
        self.assertTrue(failed == 0, msg="Not all paths found are valid!\nYou failed " + str(failed) + " cases out of " + str(len(student_solns)) + " cases.\nAn example failed case:\n" + sample_failed_case)
        
        print_success_message(test_name + "." + self._testMethodName)

    def test_optimal_paths(self, test_name, test_func, **kwargs):
        # Collect all student paths͏︌͏󠄁͏︉
        student_solns = {}
        for goal_case in self.romania_refs[test_name]:
            self.romania.reset_search()
            student_solns[goal_case] = (test_func(self.romania, goal_case[0], goal_case[1], **kwargs), self.romania.explored_nodes())

        # Check if all paths are optimal͏︌͏󠄁͏︉
        failed = 0
        sample_failed_case = ""
        for case in student_solns:
            verdict, expected_path, path_cost, expected_path_cost = self.is_optimal_path(test_name, case, student_solns[case][0])
            if not verdict:
                failed += 1
                if failed == 1:
                    sample_failed_case = "Path %s for goal nodes %s does not match reference. Path cost was %s and expected path cost was %s. Expected path is %s" % (student_solns[case][0], case, path_cost, expected_path_cost, expected_path)
        self.assertTrue(failed == 0, msg="Not all paths found were optimal!\nYou failed " + str(failed) + " cases out of " + str(len(student_solns)) + " cases.\nAn example failed case:\n" + sample_failed_case)
        
        print_success_message(test_name + "." + self._testMethodName)

    def test_explored_counts(self, test_name, test_func, **kwargs):
        # Collect all student paths͏︌͏󠄁͏︉
        student_solns = {}
        for goal_case in self.romania_refs[test_name]:
            self.romania.reset_search()
            student_solns[goal_case] = (test_func(self.romania, goal_case[0], goal_case[1], **kwargs), self.romania.explored_nodes())

        # Check if all paths are optimal͏︌͏󠄁͏︉
        failed = 0
        sample_failed_case = ""
        for case in student_solns:
            verdict, explored_cnt, expected_explored_cnt_max = self.is_allowed_explored_cnt(test_name, case, student_solns[case][1])
            if not verdict:
                failed += 1
                if failed == 1:
                    sample_failed_case = "Path %s for goal nodes %s explored more nodes than allowed maximum. Explored count was %s and max allowed count was %s" % (student_solns[case][0], case, explored_cnt, expected_explored_cnt_max)
        self.assertTrue(failed == 0, msg="Explored counts exceeded for some paths!\nYou failed " + str(failed) + " cases out of " + str(len(student_solns)) + " cases.\nAn example failed case:\n" + sample_failed_case)
        
        print_success_message(test_name + "." + self._testMethodName)

    def test_alternate_representation(self, test_name, test_func, **kwargs):
        # Setup graph͏︌͏󠄁͏︉
        with(open("romania/romania_graph2.pickle", "rb")) as romania_file:
            romania = pickle.load(romania_file)
        self.romania = ExplorableGraph(romania)

        # Setup references͏︌͏󠄁͏︉
        with(open("romania/romania_references2.pickle", "rb")) as romania_ref_file:
            self.romania_refs = pickle.load(romania_ref_file)
        
        self.romania.reset_search()
        self.test_valid_paths(test_name, test_func, **kwargs)
        self.romania.reset_search()
        self.test_optimal_paths(test_name, test_func, **kwargs)
        self.romania.reset_search()
        self.test_explored_counts(test_name, test_func, **kwargs)

# Class for testing Euclidean Distance Heuristic͏︌͏󠄁͏︉
class TestEuclideanHeuristic(unittest.TestCase):
    """Test Euclidean Heuristic implementation"""
    def __init__(self, methodName: str = ...) -> None:
        super().__init__(methodName)

        # Setup graph͏︌͏󠄁͏︉
        with(open("romania/romania_graph.pickle", "rb")) as romania_file:
            romania = pickle.load(romania_file)
        self.romania = ExplorableGraph(romania)
        self.romania.reset_search()

        # Setup references͏︌͏󠄁͏︉
        with(open("romania/romania_references.pickle", "rb")) as romania_ref_file:
            self.romania_refs = pickle.load(romania_ref_file)

    def test_euclidean_distance(self, euclidean_dist_heuristic):
        """
        Test euclidean distance calculation
        """

        # Test neighbors͏︌͏󠄁͏︉
        self.assertEqual(euclidean_dist_heuristic(self.romania, 'n', 'i'), 73.824)

        # Test non-neighbors͏︌͏󠄁͏︉
        self.assertEqual(euclidean_dist_heuristic(self.romania, 'n', 'u'), 193.569)
        
        print_success_message(self._testMethodName)

# Class for three-goal search algorithm testing͏︌͏󠄁͏︉
class TestTriSearchAlgorithms(unittest.TestCase):
    """Test standard search algorithm implementations"""
    def __init__(self, methodName: str = ...) -> None:
        super().__init__(methodName)

        # Setup graph͏︌͏󠄁͏︉
        with(open("romania/romania_graph.pickle", "rb")) as romania_file:
            romania = pickle.load(romania_file)
        self.romania = ExplorableGraph(romania)
        self.romania.reset_search()

        # Setup references͏︌͏󠄁͏︉
        with(open("romania/romania_references.pickle", "rb")) as romania_ref_file:
            self.romania_refs = pickle.load(romania_ref_file)

    @staticmethod
    def get_path_cost(graph, path):
        """
        Calculate the total cost of a path by summing edge weights

        Args:
            graph (ExplorableGraph): Graph that contains the path
            path (list(nodes)): List of nodes from src to dst

        Returns:
            Sum of edge weights in path
        """

        pairs = zip(path, path[1:])
        return sum([graph.get_edge_data(a, b)['weight'] for a, b in pairs])
    
    @staticmethod
    def is_continuous_path(graph, path):
        """
        Checks if the provided path is a continuous path

        Args:
            graph (ExplorableGraph): Undirected graph with path
            path (list(nodes)): List of nodes

        Returns:
            Boolean True if path is continuous, False otherwise
        """

        for i in range(0, len(path)-1):
            edges = networkx.edges(graph, path[i])
            if not any([path[i+1] == v for e,v in edges]):
                return False
        return True
    
    def is_valid_path(self, graph, goals, path):
        """
        Checks if the provided path (two goals) is a valid path

        Args:
            graph (ExplorableGraph): Undirected graph with path
            goals (list(nodes)): Key for the goal nodes
            path (list(nodes)): List of nodes from src to dst

        Returns:
            Boolean True if path is valid, False otherwise
        """

        # Check endpoints͏︌͏󠄁͏︉
        if not (path[0] in goals and path[-1] in goals):
            return False
        
        # Check path contains all goals͏︌͏󠄁͏︉
        for goal in goals:
            if goal not in path:
                return False
        
        # Check path continuity͏︌͏󠄁͏︉
        return self.is_continuous_path(graph, path)
    
    def is_optimal_path(self, test_name, case, path):
        """
        Check if path provided is shortest path matching references

        Args:
            test_name: Name of test
            case (tuple(node)): Goal nodes
            path (list(node)): Student's solution path
        
        Returns:
            Boolean, Path Cost, Expected Cost
        """
        
        path_cost = self.get_path_cost(self.romania, path)
        ref_path = self.romania_refs[test_name][case][0]
        # Check if ref_path is accidentally flipped͏︌͏󠄁͏︉
        if len(path) > 0 and ref_path[-1] == path[0]:
            ref_path = ref_path[::-1]
        ref_path_cost = self.get_path_cost(self.romania, ref_path)

        matches_ref = path == ref_path
        return matches_ref, ref_path, path_cost, ref_path_cost
    
    def is_allowed_explored_cnt(self, test_name, case, explored):
        """
        Check if explored is within allowed bounds of reference

        Args:
            test_name: Name of test
            case (tuple(node)): Goal nodes
            path (dict(node: int)): Student's explored

        Returns:
            Boolean, Explored Count, Expected Explored Count Max
        """

        explored_cnt = sum(explored.values())
        ref_explored_cnt_max = self.romania_refs[test_name][case][1]

        is_within_bounds = explored_cnt <= ref_explored_cnt_max
        return is_within_bounds, explored_cnt, ref_explored_cnt_max
    
    def test_valid_paths(self, test_name, test_func, **kwargs):
        # Collect all student paths͏︌͏󠄁͏︉
        student_solns = {}
        for goal_case in self.romania_refs[test_name]:
            self.romania.reset_search()
            student_solns[goal_case] = (test_func(self.romania, list(goal_case), **kwargs), self.romania.explored_nodes())

        # Check if all paths are valid͏︌͏󠄁͏︉
        failed = 0
        sample_failed_case = ""
        for case in student_solns:
            verdict = self.is_valid_path(self.romania, case, student_solns[case][0])
            if not verdict:
                failed += 1
                if failed == 1:
                    sample_failed_case = "Path %s for goal nodes '%s' is not a valid path" % (student_solns[case][0], case)
        self.assertTrue(failed == 0, msg="Not all paths found are valid!\nYou failed " + str(failed) + " cases out of " + str(len(student_solns)) + " cases.\nAn example failed case:\n" + sample_failed_case)
        
        print_success_message(test_name + "." + self._testMethodName)

    def test_optimal_paths(self, test_name, test_func, **kwargs):
        # Collect all student paths͏︌͏󠄁͏︉
        student_solns = {}
        for goal_case in self.romania_refs[test_name]:
            self.romania.reset_search()
            student_solns[goal_case] = (test_func(self.romania, list(goal_case), **kwargs), self.romania.explored_nodes())

        # Check if all paths are optimal͏︌͏󠄁͏︉
        failed = 0
        sample_failed_case = ""
        for case in student_solns:
            verdict, expected_path, path_cost, expected_path_cost = self.is_optimal_path(test_name, case, student_solns[case][0])
            if not verdict:
                failed += 1
                if failed == 1:
                    sample_failed_case = "Path %s for goal nodes %s does not match reference. Path cost was %s and expected path cost was %s. Expected path is %s" % (student_solns[case][0], case, path_cost, expected_path_cost, expected_path)
        self.assertTrue(failed == 0, msg="Not all paths found were optimal!\nYou failed " + str(failed) + " cases out of " + str(len(student_solns)) + " cases.\nAn example failed case:\n" + sample_failed_case)
        
        print_success_message(test_name + "." + self._testMethodName)

    def test_explored_counts(self, test_name, test_func, **kwargs):
        # Collect all student paths͏︌͏󠄁͏︉
        student_solns = {}
        for goal_case in self.romania_refs[test_name]:
            self.romania.reset_search()
            student_solns[goal_case] = (test_func(self.romania, list(goal_case), **kwargs), self.romania.explored_nodes())

        # Check if all paths are optimal͏︌͏󠄁͏︉
        failed = 0
        sample_failed_case = ""
        for case in student_solns:
            verdict, explored_cnt, expected_explored_cnt_max = self.is_allowed_explored_cnt(test_name, case, student_solns[case][1])
            if not verdict:
                failed += 1
                if failed == 1:
                    sample_failed_case = "Path %s for goal nodes %s explored more nodes than allowed maximum. Explored count was %s and max allowed count was %s" % (student_solns[case][0], case, explored_cnt, expected_explored_cnt_max)
        self.assertTrue(failed == 0, msg="Explored counts exceeded for some paths!\nYou failed " + str(failed) + " cases out of " + str(len(student_solns)) + " cases.\nAn example failed case:\n" + sample_failed_case)
        
        print_success_message(test_name + "." + self._testMethodName)

    def test_alternate_representation(self, test_name, test_func, **kwargs):
        # Setup graph͏︌͏󠄁͏︉
        with(open("romania/romania_graph2.pickle", "rb")) as romania_file:
            romania = pickle.load(romania_file)
        self.romania = ExplorableGraph(romania)

        # Setup references͏︌͏󠄁͏︉
        with(open("romania/romania_references2.pickle", "rb")) as romania_ref_file:
            self.romania_refs = pickle.load(romania_ref_file)
        
        self.romania.reset_search()
        self.test_valid_paths(test_name, test_func, **kwargs)
        self.romania.reset_search()
        self.test_optimal_paths(test_name, test_func, **kwargs)
        self.romania.reset_search()
        self.test_explored_counts(test_name, test_func, **kwargs)

    def get_landmarks(self, compute_landmarks):
        """Compute landmarks helper function"""
        return compute_landmarks(self.romania)