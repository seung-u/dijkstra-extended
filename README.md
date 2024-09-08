# PathNavigator Library Documentation

## Table of Contents
1. [Introduction](#introduction)
2. [Interfaces](#interfaces)
   - [IPathNavigator](#ipathnavigator)
   - [IPathNavigatorExpansion](#ipathnavigatorexpansion)
3. [PathNavigator Class](#pathnavigator-class)
   - [Constructor](#constructor)
   - [Methods](#methods)
4. [Usage Examples](#usage-examples)

## Introduction

The PathNavigator library provides functionality for finding optimal routes in a graph-like structure. It supports various path-finding operations, including finding the shortest path between two points, calculating route costs, and finding paths with required stopovers.

## Interfaces

### IPathNavigator

This interface defines the basic path navigation methods.

```python
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
```

### IPathNavigatorExpansion

This interface extends the basic functionality with methods that include a filter parameter.

```python
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
```

## PathNavigator Class

The `PathNavigator` class implements both `IPathNavigator` and `IPathNavigatorExpansion` interfaces.

### Constructor

```python
def __init__(self, edges):
    """
    Initialize the PathNavigator with a list of edges.

    Args:
    edges (list): A list of tuples representing edges in the graph.
                  Each tuple should be in the format ((node1, node2), (weight, cost))
                  or ((node1, node2), weight) where weight is used for both weight and cost.
    
    Raises:
    ValueError: If edges is empty or if edge format is invalid.
    """
```

### Methods

1. `FindOptimalRoute(self, start, end, iteration_limit)`
   - Finds the optimal route between start and end nodes.
   - Returns a list of tuples (total_distance, total_cost, path).

2. `FindOptimalStopOverRoute(self, start, end, required_nodes, iteration_limit)`
   - Finds the optimal route between start and end nodes, passing through required nodes.
   - Returns a list of tuples (total_distance, total_cost, path).

3. `CalculateDistanceCost(self, start, end, iteration_limit)`
   - Calculates the distance cost between start and end nodes.
   - Returns a list of distances.

4. `CalculateStopOverCost(self, start, end, required_nodes, iteration_limit)`
   - Calculates the cost of a route passing through required nodes.
   - Returns a list of distances.

5. `FindOptimalRouteExpansion(self, start, end, iteration_limit, filter)`
   - Similar to FindOptimalRoute, but with an additional filter parameter.

6. `FindOptimalStopOverRouteExpansion(self, start, end, required_nodes, iteration_limit, filter)`
   - Similar to FindOptimalStopOverRoute, but with an additional filter parameter.

7. `CalculateDistanceCostExpansion(self, start, end, iteration_limit, filter)`
   - Similar to CalculateDistanceCost, but with an additional filter parameter.

8. `CalculateStopOverCostExpansion(self, start, end, required_nodes, iteration_limit, filter)`
   - Similar to CalculateStopOverCost, but with an additional filter parameter.

## Usage Examples

```python
# Create a PathNavigator instance
edges = [
    (("A..B"), (5, 10)),  # Bidirectional edge between A and B with weight 5 and cost 10
    (("B.C"), 7),         # Unidirectional edge from B to C with weight and cost 7
    (("A..C"), (8, 15)),  # Bidirectional edge between A and C with weight 8 and cost 15
]
navigator = PathNavigator(edges)

# Find optimal route
optimal_route = navigator.FindOptimalRoute("A", "C", 5)
print("Optimal route:", optimal_route)

# Find optimal route with stopover
optimal_stopover_route = navigator.FindOptimalStopOverRoute("A", "C", ["B"], 5)
print("Optimal route with stopover:", optimal_stopover_route)

# Calculate distance cost
distance_cost = navigator.CalculateDistanceCost("A", "C", 5)
print("Distance cost:", distance_cost)

# Use expansion methods with a filter
def cost_filter(cost):
    return cost <= 20

optimal_route_expansion = navigator.FindOptimalRouteExpansion("A", "C", 5, cost_filter)
print("Optimal route with cost filter:", optimal_route_expansion)
```

Note: The actual output will depend on the graph structure defined by the edges.