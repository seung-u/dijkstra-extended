from abc import ABC, abstractmethod
import heapq
from itertools import product
from collections import defaultdict

# Abstract base classes defining the interface for path navigation
class IPathNavigator(ABC):
    @abstractmethod
    def FindOptimalRoute(self, start, end, iteration_limit):
        pass

    @abstractmethod
    def FindOptimalStopOverRoute(self, start, end, required_nodes, iteration_limit):
        pass

    @abstractmethod
    def CalculateDistanceCost(self, start, end, iteration_limit):
        pass

    @abstractmethod
    def CalculateStopOverCost(self, start, end, required_nodes, iteration_limit):
        pass

class IPathNavigatorExpansion(ABC):
    @abstractmethod
    def FindOptimalRouteExpansion(self, start, end, iteration_limit, filter):
        pass

    @abstractmethod
    def FindOptimalStopOverRouteExpansion(self, start, end, required_nodes, iteration_limit, filter):
        pass

    @abstractmethod
    def CalculateDistanceCostExpansion(self, start, end, iteration_limit, filter):
        pass

    @abstractmethod
    def CalculateStopOverCostExpansion(self, start, end, required_nodes, iteration_limit, filter):
        pass

class PathNavigator(IPathNavigator, IPathNavigatorExpansion):

    def __dijkstra(self, start, end, iteration_limit, filter=lambda x: True):
        """
        Implementation of Dijkstra's algorithm for finding shortest paths.
        This method is used internally by other methods to find optimal routes.
        """
        try:
            distances = {node: float('infinity') for node in self._graph}
            costs = {node: float('infinity') for node in self._graph}
            distances[start] = 0
            pq = [(0, 0, start, [start])]
            paths = []

            while pq:
                (dist, cost, current, path) = heapq.heappop(pq)
                if current == end:
                    paths.append((dist, cost, path))
                    if len(paths) == iteration_limit:
                        return paths

                if dist > distances[current] or cost > costs[current]:
                    continue

                for neighbor, (weight, edge_cost) in self._graph[current].items():
                    distance = dist + weight
                    total_cost = cost + edge_cost
                    if distance < distances[neighbor] and filter(total_cost):
                        distances[neighbor] = distance
                        costs[neighbor] = total_cost
                        heapq.heappush(pq, (distance, total_cost, neighbor, path + [neighbor]))

            return paths
        except Exception as e:
            print(f"An error occurred in __dijkstra: {str(e)}")
            return []

    def FindOptimalRoute(self, start, end, iteration_limit):
        """
        Finds the optimal route between start and end nodes.
        Returns a list of paths sorted by total distance.
        """
        try:
            nodes = [start, end]
            all_paths = []

            for i in range(len(nodes) - 1):
                paths = self.__dijkstra(nodes[i], nodes[i+1], iteration_limit)
                if not paths:
                    return []  
                all_paths.append(paths)

            complete_paths = []
            for path_combination in product(*all_paths):
                total_distance = sum(dist for dist, _, _ in path_combination)
                total_cost = sum(cost for _, cost, _ in path_combination)
                total_path = []
                for i, (_, _, path) in enumerate(path_combination):
                    total_path.extend(path if i == 0 else path[1:])
                complete_paths.append((total_distance, total_cost, total_path))

            complete_paths.sort(key=lambda x: x[0])
            return complete_paths[:iteration_limit]
        except Exception as e:
            print(f"An error occurred in FindOptimalRoute: {str(e)}")
            return []

    def FindOptimalStopOverRoute(self, start, end, required_nodes, iteration_limit):
        """
        Finds the optimal route between start and end nodes, passing through required nodes.
        Returns a list of paths sorted by total distance.
        """
        try:
            if required_nodes is None:
                required_nodes = []

            nodes = [start] + required_nodes + [end]
            all_paths = []

            for i in range(len(nodes) - 1):
                paths = self.__dijkstra(nodes[i], nodes[i+1], iteration_limit)
                if not paths:
                    return []  
                all_paths.append(paths)

            complete_paths = []
            for path_combination in product(*all_paths):
                total_distance = sum(dist for dist, _, _ in path_combination)
                total_cost = sum(cost for _, cost, _ in path_combination)
                total_path = []
                for i, (_, _, path) in enumerate(path_combination):
                    total_path.extend(path if i == 0 else path[1:])
                complete_paths.append((total_distance, total_cost, total_path))

            complete_paths.sort(key=lambda x: x[0])
            return complete_paths[:iteration_limit]
        except Exception as e:
            print(f"An error occurred in FindOptimalStopOverRoute: {str(e)}")
            return []

    def CalculateDistanceCost(self, start, end, iteration_limit):
        """
        Calculates the distance cost for optimal routes between start and end nodes.
        Returns a list of distances for the found paths.
        """
        try:
            paths = self.FindOptimalRoute(start, end, iteration_limit)
            return [path[0] for path in paths]
        except Exception as e:
            print(f"An error occurred in CalculateDistanceCost: {str(e)}")
            return []

    def CalculateStopOverCost(self, start, end, required_nodes, iteration_limit):
        """
        Calculates the distance cost for optimal routes between start and end nodes,
        passing through required nodes. Returns a list of distances for the found paths.
        """
        try:
            if required_nodes is None:
                required_nodes = []

            nodes = [start] + required_nodes + [end]
            all_paths = []

            for i in range(len(nodes) - 1):
                paths = self.__dijkstra(nodes[i], nodes[i+1], iteration_limit)
                if not paths:
                    return []  
                all_paths.append(paths)

            complete_paths = []
            for path_combination in product(*all_paths):
                total_distance = sum(dist for dist, _, _ in path_combination)
                total_cost = sum(cost for _, cost, _ in path_combination)
                total_path = []
                for i, (_, _, path) in enumerate(path_combination):
                    total_path.extend(path if i == 0 else path[1:])
                complete_paths.append((total_distance, total_cost))

            complete_paths.sort(key=lambda x: x[0])
            return [path[0] for path in complete_paths[:iteration_limit]]
        except Exception as e:
            print(f"An error occurred in CalculateStopOverCost: {str(e)}")
            return []

    def FindOptimalRouteExpansion(self, start, end, iteration_limit, filter):
        """
        Expanded version of FindOptimalRoute that includes a filter function
        to further constrain the path search.
        """
        try:
            nodes = [start, end]
            all_paths = []

            for i in range(len(nodes) - 1):
                paths = self.__dijkstra(nodes[i], nodes[i+1], iteration_limit, filter)
                if not paths:
                    return []  
                all_paths.append(paths)

            complete_paths = []
            for path_combination in product(*all_paths):
                total_distance = sum(dist for dist, _, _ in path_combination)
                total_cost = sum(cost for _, cost, _ in path_combination)
                total_path = []
                for i, (_, _, path) in enumerate(path_combination):
                    total_path.extend(path if i == 0 else path[1:])
                complete_paths.append((total_distance, total_cost, total_path))

            complete_paths.sort(key=lambda x: x[0])
            return complete_paths[:iteration_limit]
        except Exception as e:
            print(f"An error occurred in FindOptimalRouteExpansion: {str(e)}")
            return []

    def FindOptimalStopOverRouteExpansion(self, start, end, required_nodes, iteration_limit, filter):
        """
        Expanded version of FindOptimalStopOverRoute that includes a filter function
        to further constrain the path search.
        """
        try:
            if required_nodes is None:
                required_nodes = []

            nodes = [start] + required_nodes + [end]
            all_paths = []

            for i in range(len(nodes) - 1):
                paths = self.__dijkstra(nodes[i], nodes[i+1], iteration_limit, filter)
                if not paths:
                    return []  
                all_paths.append(paths)

            complete_paths = []
            for path_combination in product(*all_paths):
                total_distance = sum(dist for dist, _, _ in path_combination)
                total_cost = sum(cost for _, cost, _ in path_combination)
                total_path = []
                for i, (_, _, path) in enumerate(path_combination):
                    total_path.extend(path if i == 0 else path[1:])
                complete_paths.append((total_distance, total_cost, total_path))

            complete_paths.sort(key=lambda x: x[0])
            return complete_paths[:iteration_limit]
        except Exception as e:
            print(f"An error occurred in FindOptimalStopOverRouteExpansion: {str(e)}")
            return []

    def CalculateDistanceCostExpansion(self, start, end, iteration_limit, filter):
        """
        Expanded version of CalculateDistanceCost that includes a filter function
        to further constrain the distance calculation.
        """
        try:
            paths = self.FindOptimalRouteExpansion(start, end, iteration_limit, filter)
            return [path[0] for path in paths]
        except Exception as e:
            print(f"An error occurred in CalculateDistanceCostExpansion: {str(e)}")
            return []

    def CalculateStopOverCostExpansion(self, start, end, required_nodes, iteration_limit, filter):
        """
        Expanded version of CalculateStopOverCost that includes a filter function
        to further constrain the distance calculation.
        """
        try:
            if required_nodes is None:
                required_nodes = []

            nodes = [start] + required_nodes + [end]
            all_paths = []

            for i in range(len(nodes) - 1):
                paths = self.__dijkstra(nodes[i], nodes[i+1], iteration_limit, filter)
                if not paths:
                    return []  
                all_paths.append(paths)

            complete_paths = []
            for path_combination in product(*all_paths):
                total_distance = sum(dist for dist, _, _ in path_combination)
                total_cost = sum(cost for _, cost, _ in path_combination)
                total_path = []
                for i, (_, _, path) in enumerate(path_combination):
                    total_path.extend(path if i == 0 else path[1:])
                complete_paths.append((total_distance, total_cost))

            complete_paths.sort(key=lambda x: x[0])
            return [path[0] for path in complete_paths[:iteration_limit]]
        except Exception as e:
            print(f"An error occurred in CalculateStopOverCostExpansion: {str(e)}")
            return []

    @staticmethod
    def __convert_to_graph(edges):
        """
        Converts a list of edges to a graph representation.
        Handles both directed and undirected edges.
        """
        try:
            graph = defaultdict(dict)

            for edge in edges:
                nodes, weight_cost = edge

                if isinstance(weight_cost, tuple):
                    weight, cost = weight_cost
                else:
                    weight, cost = weight_cost, weight_cost

                if '..' in nodes:
                    start, end = nodes.split('..')
                else:
                    start, end = nodes.split('.')

                graph[start][end] = (weight, cost)
                if '..' in nodes:
                    graph[end][start] = (weight, cost)

            return dict(graph)
        except Exception as e:
            print(f"An error occurred in __convert_to_graph: {str(e)}")
            return {}

    def __init__(self, edges):
        """
        Initializes the PathNavigator with a list of edges.
        Converts the edges to a graph representation.
        """
        try:
            self._graph = self.__convert_to_graph(edges)
        except Exception as e:
            print(f"An error occurred while initializing PathNavigator: {str(e)}")
            self._graph = {}