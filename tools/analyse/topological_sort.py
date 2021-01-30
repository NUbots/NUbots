class AdjacencyList:
    """
    A class that defines a digraph

    Attr:
        inner_list: A map that defines what a vertex's edges are
        size: The number of vertices
    """

    def __init__(self):
        self.inner_list = {}
        self.size = 0

    def init_vertex(self, u):
        """
        Creates a vertex

        Args:
            u (hashable): The object to set as the vertex
        """
        if not self.inner_list.get(u):
            self.inner_list[u] = []

    def add_edge(self, u, v):
        """
        Creates an edge between two vertices. Adding the vertice u if not present.

        Args:
            u (hashable): The vertex the edge comes from
            v (hashable): The vertex the edge goes to
        """
        if not self.inner_list.get(u):
            self.inner_list[u] = []
        self.inner_list[u].append(v)

    def get(self, key):
        """
        Returns the list of edges from key

        Args:
            key (hashable): The vertex to get the edges from

        Returns:
            list(hashable): The list of vertices that the key goes to
        """
        return self.inner_list.get(key)

    def __getitem__(self, key):
        """
        Returns the list of edges from key

        Args:
            key (hashable): The vertex to get the edges from

        Returns:
            list(hashable): The list of vertices that the key goes to
        """
        return self.inner_list[key]

    def __len__(self):
        """
        Gets the size

        Returns:
            int: The size of the list
        """
        return self.size

    def __iter__(self):
        """
        Iterates through the vertices

        Returns:
            iter(hashable): An iterator over all the vertices
        """
        return iter(self.inner_list)


def topological_sort(adj):
    """
    Topological sorts the adjacency list

    Args:
        adj (AdjacencyList): The digraph to sort

    Returns:
        list(hashable): The vertices sorted
    """
    t_sorted = []
    visit = {}
    for u in adj:
        visit[u] = False

    for u in adj:
        if not visit[u]:
            _topological_sort_recurs(adj, u, t_sorted, visit)

    return t_sorted


def _topological_sort_recurs(adj, start, t_sorted, visit):
    """
    The recursive part of topological sort

    Args:
        adj (AdjacencyList): The digraph to sort
        start (hashable): The vertex to start this part of the sort at
        t_sorted (list(hashable)): The vertices sorted
        visit (dict(hashable, bool)): The list of visited vertices
    """
    visit[start] = True
    trav = adj[start]
    for trav_index in range(len(trav)):
        v = trav[trav_index]
        if not visit[v]:
            _topological_sort_recurs(adj, v, t_sorted, visit)

    t_sorted.append(start)
