///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///
#ifndef PLANNING_PATH_PLANNING_GRAPH_H
#define PLANNING_PATH_PLANNING_GRAPH_H

#include "planning/path_planning/i_graph.h"

namespace planning
{
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
    /// @copydoc IGraph::AdjacencyMatrix
    ///
    using AdjacencyMatrix = IGraph::AdjacencyMatrix;

    ///
    /// @copydoc IGraph::Edges
    ///
    using Edges = IGraph::Edges;

    ///
    /// @brief Constructor
    ///
    Graph();

    ///
    /// @copydoc IGraph::AddEdge(const Vertex& from, const Vertex& to)
    ///
    void AddEdge(const Vertex& from, const Vertex& to, const double cost = 1.0) override;

    ///
    /// @copydoc IGraph::GetAdjacentVertices(const Vertex& vertex)
    ///
    Vertices GetAdjacentVertices(const Vertex& vertex) const override;

    ///
    /// @copydoc IGraph::GetDegree(const Vertex& vertex)
    ///
    std::int32_t GetDegree(const Vertex& vertex) const override;

    ///
    /// @copydoc IGraph::GetPath(const Vertex& from, const Vertex& to)
    ///
    Vertices GetPath(const Vertex& from, const Vertex& to) override;

    ///
    /// @copydoc IGraph::IsClosedPath(const Vertex& from, const Vertex& to)
    ///

    bool IsClosedPath(const Vertex& from, const Vertex& to) const override;

    ///
    /// @copydoc IGraph::IsSimplePath(const Vertex& from, const Vertex& to)
    ///
    bool IsSimplePath(const Vertex& from, const Vertex& to) const override;

    ///
    /// @copydoc IGraph::IsCyclePath(const Vertex& from, const Vertex& to)
    ///
    bool IsCyclePath(const Vertex& from, const Vertex& to) const override;

    ///
    /// @copydoc IGraph::IsConnectedGraph()
    ///
    bool IsConnectedGraph() const override;

    ///
    /// @copydoc IGraph::IsCompleteGraph()
    ///
    bool IsCompleteGraph() const override;

    ///
    /// @copydoc IGraph::IsWeightedGraph()
    ///
    bool IsWeightedGraph() const override;

    ///
    /// @copydoc IGraph::IsDiagraph()
    ///
    bool IsDiagraph() const override;

  private:
    ///
    /// @brief List of Graph Vertices
    ///
    Vertices vertices_;

    ///
    /// @brief List of Edges
    ///
    Edges edges_;

    ///
    /// @brief Adjacency Matrix
    ///
    AdjacencyMatrix
};

}  // namespace planning

#endif  // PLANNING_PATH_PLANNING_GRAPH_H
