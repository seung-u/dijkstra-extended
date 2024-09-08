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

    def __dijkstra(self, start, end, iteration_limit, filter=lambda x: True):
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