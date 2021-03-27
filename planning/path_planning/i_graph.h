///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_I_GRAPH_H
#define PLANNING_PATH_PLANNING_I_GRAPH_H

#include <array>
#include <cstdint>
#include <list>

namespace planning
{

class IGraph
{
  public:
    ///
    /// @brief Max Vertices in the Graph
    ///
    static constexpr std::size_t kMaxVertices = 10U;

    ///
    /// @brief Each node of the graph is called a vertex. In the above graph, A, B, C, and D are the vertices of the
    /// graph.
    ///
    using Vertex = std::int32_t;

    ///
    /// @brief List of Vertices
    ///
    using Vertices = std::array<Vertex, kMaxVertices>;

    ///
    /// @brief Adjacency Matrix [kMaxVertices x kMaxVertices]
    ///
    using AdjacencyMatrix = std::array<Vertices, kMaxVertices>;

    ///
    /// @brief Link or path between two vertices
    ///
    struct Edge
    {
        Vertex from;
        Vertex to;
        double cost;
    };

    ///
    /// @brief List of all the edges
    ///
    using Edges = std::vector<Edge>;

    ///
    /// @brief Destructor
    ///
    virtual ~IGraph() = default;

    ///
    /// @brief The link or path between two vertices is called an edge. It connects two or more vertices. The different
    /// edges in the above graph are AB, BC, AD, and DC.
    ///
    /// @param from [in] - Starting Vertex
    /// @param to [in] - Ending Vertex
    /// @param cost [in] - Edge Cost/Weight. (Default: 1.0)
    ///
    virtual void AddEdge(const Vertex& from, const Vertex& to, const double cost = 1.0) = 0;

    ///
    /// @brief In a graph, if two nodes are connected by an edge then they are called adjacent nodes or neighbors. In
    /// the above graph, vertices A and B are connected by edge AB. Thus A and B are adjacent nodes.
    ///
    /// @param vertex [in] - Vertex whose adjacent vertices is requested
    ///
    /// @return adjacent verticies
    ///
    virtual Vertices GetAdjacentVertices(const Vertex& vertex) const = 0;

    ///
    /// @brief The number of edges that are connected to a particular node is called the degree of the node.
    ///
    /// @param vertex [in] - Vertex whose degree is requested
    ///
    /// @return degree of vertex
    ///
    virtual std::int32_t GetDegree(const Vertex& vertex) const = 0;

    ///
    /// @brief The sequence of nodes that we need to follow when we have to travel from one vertex to another in a graph
    /// is called the path. In our example graph, if we need to go from node A to C, then the path would be A->B->C.
    ///
    /// @param from [in] - Starting Vertex
    /// @param to [in] - Ending Vertex
    ///
    /// @return list of verticies forming path
    ///
    virtual Vertices GetPath(const Vertex& from, const Vertex& to) = 0;

    ///
    /// @brief If the initial node is the same as a terminal node, then that path is termed as the closed path.
    ///
    /// @param from [in] - Starting Vertex
    /// @param to [in] - Ending Vertex
    ///
    /// @return True if found path between from->to is closed path, False otherwise.
    ///
    virtual bool IsClosedPath(const Vertex& from, const Vertex& to) const = 0;

    ///
    /// @brief A closed path in which all the other nodes are distinct is called a simple path.
    ///
    /// @param from [in] - Starting Vertex
    /// @param to [in] - Ending Vertex
    ///
    /// @return True if found path between from->to is simple path, False otherwise.
    ///
    virtual bool IsSimplePath(const Vertex& from, const Vertex& to) const = 0;

    ///
    /// @brief A path in which there are no repeated edges or vertices and the first and last vertices are the same is
    /// called a cycle.
    ///
    /// @param from [in] - Starting Vertex
    /// @param to [in] - Ending Vertex
    ///
    /// @return True if found path between from->to is cycle path, False otherwise.
    ///
    virtual bool IsCyclePath(const Vertex& from, const Vertex& to) const = 0;

    ///
    /// @brief A connected graph is the one in which there is a path between each of the vertices. This means that there
    /// is not a single vertex which is isolated or without a connecting edge. The graph shown above is a connected
    /// graph.
    ///
    /// @return True if graph is connected graph, False otherwise.
    ///
    virtual bool IsConnectedGraph() const = 0;

    ///
    /// @brief A graph in which each node is connected to another is called the Complete graph. If N is the total number
    /// of nodes in a graph then the complete graph contains N(N-1)/2 number of edges.
    ///
    /// @return True if graph is complete graph, False otherwise.
    ///
    virtual bool IsCompleteGraph() const = 0;

    ///
    /// @brief A positive value assigned to each edge indicating its length (distance between the vertices connected by
    /// an edge) is called weight. The graph containing weighted edges is called a weighted graph. The weight of an edge
    /// e is denoted by w(e) and it indicates the cost of traversing an edge.
    ///
    /// @return True if graph is weighted graph, False otherwise.
    ///
    virtual bool IsWeightedGraph() const = 0;

    ///
    /// @brief A digraph is a graph in which every edge is associated with a specific direction and the traversal can be
    /// done in specified direction only.
    ///
    /// @return True if graph is diagraph, False otherwise.
    ///
    virtual bool IsDiagraph() const = 0;
};

}  // namespace planning
#endif  // PLANNING_PATH_PLANNING_I_GRAPH_H
