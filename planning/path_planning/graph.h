///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_GRAPH_H
#define PLANNING_PATH_PLANNING_GRAPH_H

#include "planning/path_planning/i_graph.h"

#include <list>
#include <unordered_map>

namespace planning
{

template <std::size_t kMaxVertices>
class Graph : public IGraph
{
  public:
    ///
    /// @copydoc IGraph::Vertex
    ///
    using Vertex = IGraph::Vertex;

    ///
    /// @copydoc IGraph::Vertices
    ///
    using Vertices = IGraph::Vertices;

    ///
    /// @brief Link or path between two Vertex. (aka Edge)
    ///
    struct Edge
    {
        Vertex from;
        Vertex to;
        double cost;
    };

    ///
    /// @brief List of Adjacency
    ///
    using Edges = std::list<Edge>;

    ///
    /// @brief Adjacency List to represent Graph
    ///
    using AdjacencyList = std::unordered_map<Vertex, Edge>;

    ///
    /// @brief Constructor
    ///
    Graph() : adjacency_list_{} {}

    ///
    /// @copydoc IGraph::AddEdge(const Vertex& from, const Vertex& to)
    ///
    void AddEdge(const Vertex& from, const Vertex& to, const double cost = 1.0) override
    {
        if (!HasVertex(from))
        {
            adjacency_list_[from] = Edge{from, to, cost};
        }
    }

    ///
    /// @copydoc IGraph::GetAdjacentVertices(const Vertex& vertex)
    ///
    Vertices GetAdjacentVertices(const Vertex& vertex) const override
    {
        Vertices vertices{};

        if (HasVertex(vertex))
        {
            const auto edge = adjacency_list_.at(vertex);
            vertices.push_back(edge.from);
            vertices.push_back(edge.to);
        }

        return vertices;
    }

    ///
    /// @copydoc IGraph::GetDegree(const Vertex& vertex)
    ///
    std::int32_t GetDegree(const Vertex& vertex) const override { return 0; }

    ///
    /// @copydoc IGraph::GetPath(const Vertex& from, const Vertex& to)
    ///
    Vertices GetPath(const Vertex& from, const Vertex& to) override { return Vertices{}; }

    ///
    /// @copydoc IGraph::IsClosedPath(const Vertex& from, const Vertex& to)
    ///

    bool IsClosedPath(const Vertex& from, const Vertex& to) const override { return false; }

    ///
    /// @copydoc IGraph::IsSimplePath(const Vertex& from, const Vertex& to)
    ///
    bool IsSimplePath(const Vertex& from, const Vertex& to) const override { return false; }

    ///
    /// @copydoc IGraph::IsCyclePath(const Vertex& from, const Vertex& to)
    ///
    bool IsCyclePath(const Vertex& from, const Vertex& to) const override { return false; }

    ///
    /// @copydoc IGraph::IsConnectedGraph()
    ///
    bool IsConnectedGraph() const override { return false; }

    ///
    /// @copydoc IGraph::IsCompleteGraph()
    ///
    bool IsCompleteGraph() const override { return false; }

    ///
    /// @copydoc IGraph::IsWeightedGraph()
    ///
    bool IsWeightedGraph() const override { return false; }

    ///
    /// @copydoc IGraph::IsDiagraph()
    ///
    bool IsDiagraph() const override { return false; }

  private:
    inline bool HasVertex(const Vertex& vertex) const
    {
        return (adjacency_list_.find(vertex) != adjacency_list_.end());
    }

    ///
    /// @brief Adjacency List
    ///
    AdjacencyList adjacency_list_;
};

}  // namespace planning

#endif  // PLANNING_PATH_PLANNING_GRAPH_H
