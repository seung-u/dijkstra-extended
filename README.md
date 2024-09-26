# Path Navigator Library Documentation

## Overview
The Path Navigator library provides an interface and implementation for finding optimal routes in a graph. It supports both direct routes and routes with stopovers, utilizing Dijkstra's algorithm for pathfinding.

## Classes

### IPathNavigator
An abstract base class defining the interface for path navigation.

#### Methods
- `find_optimal_route(start, end, iteration_limit)`: Finds the optimal route between two nodes.
- `find_optimal_stopover_route(start, end, required_nodes, iteration_limit)`: Finds the optimal route with specified stopover nodes.
- `calculate_distance_cost(start, end, iteration_limit)`: Calculates the distance cost for the optimal route.
- `calculate_stopover_cost(start, end, required_nodes, iteration_limit)`: Calculates the stopover cost for the optimal route.

### IPathNavigatorExpansion
An abstract base class extending the path navigation interface with filtering capabilities.

#### Methods
- `find_optimal_route_expansion(start, end, iteration_limit, filter)`: Finds the optimal route with a filter.
- `find_optimal_stopover_route_expansion(start, end, required_nodes, iteration_limit, filter)`: Finds the optimal stopover route with a filter.
- `calculate_distance_cost_expansion(start, end, iteration_limit, filter)`: Calculates the distance cost with a filter.
- `calculate_stopover_cost_expansion(start, end, required_nodes, iteration_limit, filter)`: Calculates the stopover cost with a filter.

### PathNavigator
Implements both `IPathNavigator` and `IPathNavigatorExpansion`.

#### Constructor
- `__init__(edges)`: Initializes the PathNavigator with a list of edges.

#### Private Methods
- `__convert_to_graph(edges)`: Converts a list of edges to a graph representation.
- `__find_paths(nodes, iteration_limit, filter)`: Finds paths between a list of nodes.
- `__combine_paths(all_paths, iteration_limit)`: Combines multiple paths into complete paths.
- `__dijkstra(start, end, iteration_limit, filter)`: Implements Dijkstra's algorithm for finding shortest paths.

#### Public Methods
- `find_optimal_route(start, end, iteration_limit)`: Finds the optimal route between two nodes.
- `find_optimal_stopover_route(start, end, required_nodes, iteration_limit)`: Finds the optimal route with specified stopover nodes.
- `calculate_distance_cost(start, end, iteration_limit)`: Calculates the distance cost for the optimal route.
- `calculate_stopover_cost(start, end, required_nodes, iteration_limit)`: Calculates the stopover cost for the optimal route.
- `find_optimal_route_expansion(start, end, iteration_limit, filter)`: Finds the optimal route with a filter.
- `find_optimal_stopover_route_expansion(start, end, required_nodes, iteration_limit, filter)`: Finds the optimal stopover route with a filter.
- `calculate_distance_cost_expansion(start, end, iteration_limit, filter)`: Calculates the distance cost with a filter.
- `calculate_stopover_cost_expansion(start, end, required_nodes, iteration_limit, filter)`: Calculates the stopover cost with a filter.

## Usage Example
```python
edges = [
(("A", "B"), (1, 2)),
(("B", "C"), (2, 3)),
(("A", "C"), (4, 1)),
]
navigator = PathNavigator(edges)
optimal_route = navigator.find_optimal_route("A", "C", 1)
print(optimal_route)
```

## Error Handling
The library prints an error message if there is an issue parsing edges during graph conversion.


## Dependencies
- `heapq`: For priority queue implementation.
- `itertools`: For combinatorial functions.
- `collections`: For defaultdict usage.
=======
All methods include try-except blocks to catch and print any exceptions that occur during execution. In case of an error, an empty list or dictionary is typically returned.

## Notes

- The library uses Dijkstra's algorithm internally for pathfinding.
- The expanded methods allow for more fine-grained control over path selection through the use of custom filter functions.
- The graph is represented internally as a dictionary of dictionaries, allowing for efficient lookups.


> this readme is written by claude 3.5 sonnet
