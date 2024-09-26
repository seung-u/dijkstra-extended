from abc import ABC, abstractmethod
import heapq
from itertools import product
from collections import defaultdict

# Abstract base classes defining the interface for path navigation
class IPathNavigator(ABC):
    @abstractmethod
    def find_optimal_route(self, start, end, iteration_limit):
        pass

    @abstractmethod
    def find_optimal_stopover_route(self, start, end, required_nodes, iteration_limit):
        pass

    @abstractmethod
    def calculate_distance_cost(self, start, end, iteration_limit):
        pass

    @abstractmethod
    def calculate_stopover_cost(self, start, end, required_nodes, iteration_limit):
        pass

class IPathNavigatorExpansion(ABC):
    @abstractmethod
    def find_optimal_route_expansion(self, start, end, iteration_limit, filter):
        pass

    @abstractmethod
    def find_optimal_stopover_route_expansion(self, start, end, required_nodes, iteration_limit, filter):
        pass

    @abstractmethod
    def calculate_distance_cost_expansion(self, start, end, iteration_limit, filter):
        pass

    @abstractmethod
    def calculate_stopover_cost_expansion(self, start, end, required_nodes, iteration_limit, filter):
        pass

class PathNavigator(IPathNavigator, IPathNavigatorExpansion):

    def __init__(self, edges):
        """
        Initializes the PathNavigator with a list of edges.
        Converts the edges to a graph representation.
        """
        self._graph = self.__convert_to_graph(edges)
    
    @staticmethod
    def __convert_to_graph(edges):
        """
        Converts a list of edges to a graph representation.
        """
        graph = defaultdict(dict)
        for edge in edges:
            try:
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
            except ValueError:
                print(f"Error parsing edge: {edge}")
        return dict(graph)
    
    def __find_paths(self, nodes, iteration_limit, filter):
        all_paths = []
        for i in range(len(nodes) - 1):
            paths = self.__dijkstra(nodes[i], nodes[i + 1], iteration_limit, filter)
            if not paths:
                return []  
            all_paths.append(paths)
        return all_paths

    def __combine_paths(self, all_paths, iteration_limit):
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
    
    def __dijkstra(self, start, end, iteration_limit, filter=lambda x: True):
        """
        Implementation of Dijkstra's algorithm for finding shortest paths.
        """
        distances = {node: float('infinity') for node in self._graph}
        costs = {node: float('infinity') for node in self._graph}
        distances[start] = 0
        pq = [(0, 0, start, [start])]
        paths = []

        while pq:
            dist, cost, current, path = heapq.heappop(pq)
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

    def find_optimal_route(self, start, end, iteration_limit):
        nodes = [start, end]
        all_paths = self.__find_paths(nodes, iteration_limit, filter=lambda x: True)
        return self.__combine_paths(all_paths, iteration_limit)
    
    def find_optimal_stopover_route(self, start, end, required_nodes, iteration_limit):
        nodes = [start] + (required_nodes or []) + [end]
        all_paths = self.__find_paths(nodes, iteration_limit, filter=lambda x: True)
        return self.__combine_paths(all_paths, iteration_limit)
    
    def calculate_distance_cost(self, start, end, iteration_limit):
        paths = self.find_optimal_route(start, end, iteration_limit)
        return [path[0] for path in paths]
    
    def calculate_stopover_cost(self, start, end, required_nodes, iteration_limit):
        paths = self.find_optimal_stopover_route(start, end, required_nodes, iteration_limit)
        return [path[0] for path in paths]
    
    def find_optimal_route_expansion(self, start, end, iteration_limit, filter):
        nodes = [start, end]
        all_paths = self.__find_paths(nodes, iteration_limit, filter)
        return self.__combine_paths(all_paths, iteration_limit)
    
    def find_optimal_stopover_route_expansion(self, start, end, required_nodes, iteration_limit, filter):
        nodes = [start] + (required_nodes or []) + [end]
        all_paths = self.__find_paths(nodes, iteration_limit, filter)
        return self.__combine_paths(all_paths, iteration_limit)
    
    def calculate_distance_cost_expansion(self, start, end, iteration_limit, filter):
        paths = self.find_optimal_route_expansion(start, end, iteration_limit, filter)
        return [path[0] for path in paths]
    
    def calculate_stopover_cost_expansion(self, start, end, required_nodes, iteration_limit, filter):
        paths = self.find_optimal_stopover_route_expansion(start, end, required_nodes, iteration_limit, filter)
        return [path[0] for path in paths]
