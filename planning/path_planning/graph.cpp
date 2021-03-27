///
/// @file
/// @copyright Copyright (C) 2021. MIT License.
///

#include "planning/path_planning/graph.h"

namespace planning
{
Graph::Graph() : vertices_{}, edges_{} {}

void Graph::AddEdge(const Vertex& from, const Vertex& to, const double cost)
{
    const Edge edge{from, to, cost};
    edges_.push_back(edge);

    
}

Graph::Vertices Graph::GetAdjacentVertices(const Vertex& vertex) const
{
    return vertices_;
}

std::int32_t Graph::GetDegree(const Vertex& vertex) const
{
    return 0;
}

Graph::Vertices Graph::GetPath(const Vertex& from, const Vertex& to)
{
    return vertices_;
}

bool Graph::IsClosedPath(const Vertex& from, const Vertex& to) const
{
    return false;
}

bool Graph::IsSimplePath(const Vertex& from, const Vertex& to) const
{
    return false;
}

bool Graph::IsCyclePath(const Vertex& from, const Vertex& to) const
{
    return false;
}

bool Graph::IsConnectedGraph() const
{
    return false;
}

bool Graph::IsCompleteGraph() const
{
    return false;
}

bool Graph::IsWeightedGraph() const
{
    return false;
}

bool Graph::IsDiagraph() const
{
    return false;
}

}  // namespace planning
