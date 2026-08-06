#include "mapping/atcdt.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <cstdint>

#include <set>

// ==========================================================
// TopologicalMap
// ==========================================================

ATCDT::TopologicalMap::TopologicalMap()
: node_count(0)
{
}

// ==========================================================
// Utilities
// ==========================================================

std::vector<std::pair<int, int>>
ATCDT::TopologicalMap::edges() const
{
    std::set<std::pair<int, int>> visited;

    std::vector<std::pair<int, int>> edges;

    for (const auto &[a, neighbors] : graph)
    {
        for (const auto &[b, age] : neighbors)
        {
            (void)age;

            if (visited.count({b, a}))
                continue;

            visited.insert({a, b});
            edges.emplace_back(a, b);
        }
    }

    return edges;
}

// ==========================================================
// Node Operations
// ==========================================================

int ATCDT::TopologicalMap::addNode(
    const Eigen::Vector3f &position)
{
    positions.push_back(position);

    normals.emplace_back(
        Eigen::Vector3f::Zero());

    traversability.push_back(0);

    int node_id = node_count;
    ++node_count;

    return node_id;
}

// ==========================================================
// Edge Operations
// ==========================================================

bool ATCDT::TopologicalMap::hasEdge(
    int a,
    int b) const
{
    auto it = graph.find(a);

    if (it == graph.end())
        return false;

    return it->second.find(b) != it->second.end();
}

void ATCDT::TopologicalMap::addEdge(
    int a,
    int b)
{
    graph[a][b] = 0;
    graph[b][a] = 0;
}

void ATCDT::TopologicalMap::removeEdge(
    int a,
    int b)
{
    graph[a].erase(b);
    graph[b].erase(a);
}

std::vector<int> ATCDT::TopologicalMap::neighbors(
    int node) const
{
    std::vector<int> result;

    auto it = graph.find(node);

    if (it == graph.end())
        return result;

    for (const auto &[neighbor, age] : it->second)
    {
        (void)age;
        result.push_back(neighbor);
    }

    return result;
}

// ==========================================================
// Utilities
// ==========================================================

std::vector<float> ATCDT::TopologicalMap::edgeAges() const
{
    std::set<std::pair<int, int>> visited;

    std::vector<float> ages;

    for (const auto &[a, neighbors] : graph)
    {
        for (const auto &[b, age] : neighbors)
        {
            if (visited.count({b, a}))
                continue;

            visited.insert({a, b});

            ages.push_back(
                static_cast<float>(age));
        }
    }

    return ages;
}

int ATCDT::TopologicalMap::numEdges() const
{
    std::set<std::pair<int, int>> visited;

    int count = 0;

    for (const auto &[a, neighbors] : graph)
    {
        for (const auto &[b, age] : neighbors)
        {
            (void)age;

            if (visited.count({b, a}))
                continue;

            visited.insert({a, b});

            ++count;
        }
    }

    return count;
}


// ==========================================================
// Constructor
// ==========================================================

ATCDT::ATCDT(
    float vigilance,
    int lambda_points)
: vigilance(vigilance),
  lambda_points(lambda_points)
{
}

// ==========================================================
// samplePoints()
// ==========================================================

std::vector<Eigen::Vector3f> ATCDT::samplePoints(
    const std::vector<Eigen::Vector3f> &points) const
{
    if (points.size() <=
        static_cast<size_t>(lambda_points))
    {
        return points;
    }

    std::vector<int> indices(points.size());

    std::iota(
        indices.begin(),
        indices.end(),
        0);

    std::random_device rd;
    std::mt19937 gen(rd());

    std::shuffle(
        indices.begin(),
        indices.end(),
        gen);

    std::vector<Eigen::Vector3f> sampled;

    sampled.reserve(lambda_points);

    for (int i = 0; i < lambda_points; ++i)
    {
        sampled.push_back(
            points[indices[i]]);
    }

    return sampled;
}

// ==========================================================
// addNode()
// ==========================================================

int ATCDT::addNode(
    const Eigen::Vector3f &point)
{
    int node_id =
        map.addNode(point);

    winner_count.push_back(0);

    return node_id;
}

// ==========================================================
// winnerSearch()
// ==========================================================

ATCDT::WinnerResult ATCDT::winnerSearch(
    const Eigen::Vector3f &point) const
{
    if (map.node_count == 0)
    {
        return {
            -1,
            -1,
            std::numeric_limits<float>::infinity(),
            std::numeric_limits<float>::infinity()
        };
    }

    if (map.node_count == 1)
    {
        float d =
            (map.positions[0] - point).norm();

        return {
            0,
            -1,
            d,
            std::numeric_limits<float>::infinity()
        };
    }

    int s1 = -1;
    int s2 = -1;

    float d1 =
        std::numeric_limits<float>::infinity();

    float d2 =
        std::numeric_limits<float>::infinity();

    for (int i = 0; i < map.node_count; ++i)
    {
        float d =
            (map.positions[i] - point).norm();

        if (d < d1)
        {
            d2 = d1;
            s2 = s1;

            d1 = d;
            s1 = i;
        }
        else if (d < d2)
        {
            d2 = d;
            s2 = i;
        }
    }

    return {s1, s2, d1, d2};
}

// ==========================================================
// updateExistingNode()
// ==========================================================

void ATCDT::updateExistingNode(
    const Eigen::Vector3f &point,
    int winner)
{
    winner_count[winner]++;

    float lr =
        1.0f /
        (10.0f * winner_count[winner]);

    map.positions[winner] +=
        lr *
        (point - map.positions[winner]);

    for (int neighbor :
         map.neighbors(winner))
    {
        winner_count[neighbor]++;

        lr =
            1.0f /
            (100.0f * winner_count[neighbor]);

        map.positions[neighbor] +=
            lr *
            (point - map.positions[neighbor]);
    }
}

// ==========================================================
// updateEdge()
// ==========================================================

void ATCDT::updateEdge(
    int s1,
    int s2)
{
    for (int neighbor :
         map.neighbors(s1))
    {
        map.graph[s1][neighbor]++;
        map.graph[neighbor][s1]++;
    }

    if (map.hasEdge(s1, s2))
    {
        map.graph[s1][s2] = 0;
        map.graph[s2][s1] = 0;
    }
    else
    {
        map.addEdge(s1, s2);
    }
}

// ==========================================================
// removeOldEdges()
// ==========================================================

void ATCDT::removeOldEdges(
    float gmax)
{
    std::vector<std::pair<int,int>>
        remove_edges;

    std::set<std::pair<int,int>>
        visited;

    for (const auto &[a, neighbors] :
         map.graph)
    {
        for (const auto &[b, age] :
             neighbors)
        {
            if (visited.count({b,a}))
                continue;

            visited.insert({a,b});

            if (age > gmax)
            {
                deleted_edge_ages.push_back(age);

                remove_edges.emplace_back(
                    a,
                    b);
            }
        }
    }

    for (const auto &[a,b] :
         remove_edges)
    {
        map.removeEdge(a,b);
    }
}

// ==========================================================
// processFrame()
// ==========================================================

void ATCDT::processFrame(
    const std::vector<Eigen::Vector3f> &point_cloud)
{
    auto sampled =
        samplePoints(point_cloud);

    for (const auto &point :
         sampled)
    {
        auto result =
            winnerSearch(point);

        //
        // Case (a)
        //

        if (result.d1 > vigilance)
        {
            addNode(point);
            continue;
        }

        //
        // Case (b)
        //

        updateExistingNode(
            point,
            result.s1);

        //
        // Case (c)
        //

        if (result.d2 <= vigilance)
        {
            updateEdge(
                result.s1,
                result.s2);
        }
    }

    float gmax =
        computeGmax();

    removeOldEdges(gmax);
}

// ==========================================================
// computeGthr()
// ==========================================================

float ATCDT::computeGthr() const
{
    auto ages = map.edgeAges();

    if (ages.size() < 4)
    {
        return std::numeric_limits<float>::infinity();
    }

    std::sort(
        ages.begin(),
        ages.end());

    auto percentile = [&](float p)
    {
        float index = p * (ages.size() - 1);

        size_t lower =
            static_cast<size_t>(std::floor(index));

        size_t upper =
            static_cast<size_t>(std::ceil(index));

        if (lower == upper)
        {
            return ages[lower];
        }

        float weight =
            index - lower;

        return ages[lower] * (1.0f - weight)
             + ages[upper] * weight;
    };

    float q1 = percentile(0.25f);
    float q3 = percentile(0.75f);

    float iqr = q3 - q1;

    return q3 + iqr;
}

// ==========================================================
// computeGmax()
// ==========================================================

float ATCDT::computeGmax() const
{
    auto current =
        map.edgeAges();

    float gthr =
        computeGthr();

    if (deleted_edge_ages.empty())
    {
        return gthr;
    }

    float gdel =
        std::accumulate(
            deleted_edge_ages.begin(),
            deleted_edge_ages.end(),
            0.0f)
        /
        deleted_edge_ages.size();

    float total =
        current.size()
        +
        deleted_edge_ages.size();

    float weight =
        deleted_edge_ages.size()
        /
        total;

    float gmax =
        gdel * weight +
        gthr * (1.0f - weight);

    return gmax;
}