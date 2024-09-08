from abc import ABC, abstractmethod
import heapq
from itertools import product
from collections import defaultdict

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
    """
    PathNavigator implements both IPathNavigator and IPathNavigatorExpansion interfaces.
    It provides methods for finding optimal routes, calculating distances and costs,
    and handling routes with required stopovers. The class uses Dijkstra's algorithm
    for pathfinding and supports both regular and expanded functionality with custom filters.
    """

    def __dijkstra(self, start, end, iteration_limit, filter=lambda x: True):
        try:
            distances = {start: 0}
            previous = {}
            nodes = []
            heapq.heappush(nodes, (0, start))

            iterations = 0
            while nodes and iterations < iteration_limit:
                current_distance, current_node = heapq.heappop(nodes)
                
                if current_node == end:
                    path = []
                    while current_node:
                        path.append(current_node)
                        current_node = previous.get(current_node)
                    return path[::-1]

                if current_distance > distances.get(current_node, float('inf')):
                    continue

                for neighbor, weight in self.graph[current_node].items():
                    if not filter(neighbor):
                        continue
                    distance = current_distance + weight
                    if distance < distances.get(neighbor, float('inf')):
                        distances[neighbor] = distance
                        previous[neighbor] = current_node
                        heapq.heappush(nodes, (distance, neighbor))

                iterations += 1

            raise ValueError(f"Path not found within {iteration_limit} iterations")

        except KeyError as e:
            raise ValueError(f"Node not found in graph: {e}")
        except TypeError as e:
            raise ValueError(f"Invalid data type in graph: {e}")
        except Exception as e:
            raise RuntimeError(f"An unexpected error occurred during pathfinding: {e}")

    def FindOptimalRoute(self, start, end, iteration_limit):
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

    def FindOptimalStopOverRoute(self, start, end, required_nodes, iteration_limit):
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

    def CalculateDistanceCost(self, start, end, iteration_limit):
        paths = self.FindOptimalRoute(start, end, iteration_limit)
        return [path[0] for path in paths] 

    def CalculateStopOverCost(self, start, end, required_nodes, iteration_limit):
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

    def FindOptimalRouteExpansion(self, start, end, iteration_limit, filter):
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

    def FindOptimalStopOverRouteExpansion(self, start, end, required_nodes, iteration_limit, filter):
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

    def CalculateDistanceCostExpansion(self, start, end, iteration_limit, filter):
        paths = self.FindOptimalRouteExpansion(start, end, iteration_limit, filter)
        return [path[0] for path in paths]

    def CalculateStopOverCostExpansion(self, start, end, required_nodes, iteration_limit, filter):
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

    @staticmethod
    def __convert_to_graph(edges):
        graph = defaultdict(dict)

        for edge in edges:
            if not isinstance(edge, tuple) or len(edge) != 2:
                raise ValueError(f"Invalid edge format: {edge}")

            nodes, weight_cost = edge

            if isinstance(weight_cost, tuple):
                if len(weight_cost) != 2:
                    raise ValueError(f"Invalid weight_cost format: {weight_cost}")
                weight, cost = weight_cost
            else:
                weight = cost = weight_cost

            if '..' in nodes:  
                start, end = nodes.split('..')
            elif '.' in nodes:  
                start, end = nodes.split('.')
            else:
                raise ValueError(f"Invalid node format: {nodes}")

            graph[start][end] = (weight, cost)
            if '..' in nodes:  
                graph[end][start] = (weight, cost)

        return dict(graph)

    def __init__(self, edges):
        if not edges:
            raise ValueError("Edges cannot be empty")
        self._graph = self.__convert_to_graph(edges)