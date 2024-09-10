# PathNavigator Library Documentation

## Overview

The PathNavigator library provides classes and methods for finding optimal routes in a graph, with support for required stopovers and custom filtering. It implements Dijkstra's algorithm for pathfinding and offers both basic and expanded functionality.

## Classes

### IPathNavigator (Abstract Base Class)

Defines the basic interface for path navigation.

#### Methods:

- `FindOptimalRoute(start, end, iteration_limit)`
- `FindOptimalStopOverRoute(start, end, required_nodes, iteration_limit)`
- `CalculateDistanceCost(start, end, iteration_limit)`
- `CalculateStopOverCost(start, end, required_nodes, iteration_limit)`

### IPathNavigatorExpansion (Abstract Base Class)

Defines the expanded interface for path navigation with additional filtering capabilities.

#### Methods:

- `FindOptimalRouteExpansion(start, end, iteration_limit, filter)`
- `FindOptimalStopOverRouteExpansion(start, end, required_nodes, iteration_limit, filter)`
- `CalculateDistanceCostExpansion(start, end, iteration_limit, filter)`
- `CalculateStopOverCostExpansion(start, end, required_nodes, iteration_limit, filter)`

### PathNavigator

Implements both `IPathNavigator` and `IPathNavigatorExpansion` interfaces.

#### Constructor:

```python
PathNavigator(edges)
```

- `edges`: A list of edges defining the graph. Each edge is represented as a tuple `(nodes, weight_cost)`, where `nodes` is a string in the format "start.end" for directed edges or "start..end" for undirected edges, and `weight_cost` is either a single value (used for both weight and cost) or a tuple `(weight, cost)`.

#### Methods:

1. `FindOptimalRoute(start, end, iteration_limit)`
   - Finds the optimal route between start and end nodes.
   - Returns a list of paths sorted by total distance.

2. `FindOptimalStopOverRoute(start, end, required_nodes, iteration_limit)`
   - Finds the optimal route between start and end nodes, passing through required nodes.
   - Returns a list of paths sorted by total distance.

3. `CalculateDistanceCost(start, end, iteration_limit)`
   - Calculates the distance cost for optimal routes between start and end nodes.
   - Returns a list of distances for the found paths.

4. `CalculateStopOverCost(start, end, required_nodes, iteration_limit)`
   - Calculates the distance cost for optimal routes between start and end nodes, passing through required nodes.
   - Returns a list of distances for the found paths.

5. `FindOptimalRouteExpansion(start, end, iteration_limit, filter)`
   - Expanded version of FindOptimalRoute that includes a filter function to further constrain the path search.

6. `FindOptimalStopOverRouteExpansion(start, end, required_nodes, iteration_limit, filter)`
   - Expanded version of FindOptimalStopOverRoute that includes a filter function to further constrain the path search.

7. `CalculateDistanceCostExpansion(start, end, iteration_limit, filter)`
   - Expanded version of CalculateDistanceCost that includes a filter function to further constrain the distance calculation.

8. `CalculateStopOverCostExpansion(start, end, required_nodes, iteration_limit, filter)`
   - Expanded version of CalculateStopOverCost that includes a filter function to further constrain the distance calculation.

## Usage Example

```python
# Create a PathNavigator instance
edges = [
    ("A.B", (5, 10)),
    ("B.C", (3, 6)),
    "A..C", 7  # Undirected edge with weight and cost both set to 7
]
navigator = PathNavigator(edges)

# Find optimal route
optimal_route = navigator.FindOptimalRoute("A", "C", 1)
print(optimal_route)

# Find optimal route with stopover
stopover_route = navigator.FindOptimalStopOverRoute("A", "C", ["B"], 1)
print(stopover_route)

# Use expanded functionality with a custom filter
def custom_filter(cost):
    return cost <= 15

expanded_route = navigator.FindOptimalRouteExpansion("A", "C", 1, custom_filter)
print(expanded_route)
```

## Error Handling

All methods include try-except blocks to catch and print any exceptions that occur during execution. In case of an error, an empty list or dictionary is typically returned.

## Notes

- The library uses Dijkstra's algorithm internally for pathfinding.
- The expanded methods allow for more fine-grained control over path selection through the use of custom filter functions.
- The graph is represented internally as a dictionary of dictionaries, allowing for efficient lookups.


> this readme is written by claude 3.5 sonnet
