import graph

if __name__ == '__main__':
    edges = [
        ('Seoul..Incheon', (1, 100)),
        ('Seoul..Suwon', (1, 40)),
        ('Seoul..Wonju', (2, 100)),
        ('Seoul..Daejeon', (2, 100)),
        ('Suwon..Daejeon', (2, 100)),
        ('Suwon..Gangneung', (3, 100)),
        ('Wonju..Gangneung', (2, 100)),
        ('Wonju..Daejeon', (3, 100)),
        ('Gangneung..Daegu', (4, 100)),
        ('Daejeon..Daegu', (1, 80)),
        ('Daejeon..Gwangju', (2, 40)),
        ('Daegu..Busan', (1, 50)),
        ('Daegu..Gwangju', (3, 100)),
        ('Gwangju..Busan', (3, 30)),
        ('Gwangju..Jeju', (4, 60)),
        ('Busan..Jeju', (5, 70)),
        ('Daejeon..Incheon', (3, 30)),
        ('Incheon.Busan', (4, 20))
    ]
    navigator = graph.PathNavigator(edges)
    require_nodes = ['Jeju']

    res = navigator.FindOptimalStopOverRoute('Seoul', 'Busan', required_nodes=require_nodes, iteration_limit=3)
    res2 = navigator.FindOptimalRoute('Seoul', 'Busan', iteration_limit=3)
    res3 = navigator.CalculateDistanceCost('Seoul', 'Busan', iteration_limit=3)
    res4 = navigator.CalculateStopOverCost('Seoul', 'Busan', required_nodes=require_nodes, iteration_limit=3)

    res5 = navigator.FindOptimalStopOverRouteExpansion('Seoul', 'Busan', required_nodes=require_nodes, iteration_limit=3, filter=lambda x: x <= 810)
    res6 = navigator.FindOptimalRouteExpansion('Seoul', 'Busan', iteration_limit=3, filter=lambda x: x <= 810)
    res7 = navigator.CalculateDistanceCostExpansion('Seoul', 'Busan', iteration_limit=3, filter=lambda x: x <= 810)
    res8 = navigator.CalculateStopOverCostExpansion('Seoul', 'Busan', required_nodes=require_nodes, iteration_limit=3, filter=lambda x: x <= 810)

    print("Original methods results:")
    print(res)
    print(res2)
    print(res3)
    print(res4)

    print("\nNew methods with cost constraint results:")
    print(res5)
    print(res6)
    print(res7)
    print(res8)