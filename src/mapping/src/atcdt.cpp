#include "mapping/atcdt.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <cstdint>

#include <set>
#include <Eigen/Eigenvalues>

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

    slope_angles.push_back(0.0f);
    traversable.push_back(false);
    contour.push_back(false);

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
    graph[a][b] = 1;
    graph[b][a] = 1;
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
    int lambda_points,
    float max_slope,
    float contour_gap,
    const Eigen::Vector3f &gravity,
    bool exclude_untraversable_from_contour)
: vigilance(vigilance),
  lambda_points(lambda_points),
  max_slope(max_slope),
  contour_gap(contour_gap),
  gravity(gravity),
  exclude_untraversable_from_contour(exclude_untraversable_from_contour)
{
    if (this->gravity.norm() > 1e-6f)
        this->gravity.normalize();
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

    winner_count.push_back(1);

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
// updateWinner()
// ==========================================================

void ATCDT::updateWinner(
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

}

void ATCDT::updateNeighbors(
    const Eigen::Vector3f &point,
    int winner)
{
    for (int neighbor : map.neighbors(winner))
    {
        const float lr =
            1.0f / (100.0f * winner_count[neighbor]);

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
        map.graph[s1][s2] = 1;
        map.graph[s2][s1] = 1;
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
    int s1,
    float gmax)
{
    std::vector<std::pair<int,int>>
        remove_edges;

    for (const auto &[neighbor, age] : map.graph[s1])
    {
        if (age > gmax)
        {
            deleted_edge_ages.push_back(static_cast<float>(age));
            remove_edges.emplace_back(s1, neighbor);
        }
    }

    for (const auto &[a, b] : remove_edges)
        map.removeEdge(a, b);
}

// ==========================================================
// processFrame()
// ==========================================================

void ATCDT::processFrame(
    const std::vector<Eigen::Vector3f> &point_cloud)
{
    auto sampled =
        samplePoints(point_cloud);

    if (map.node_count == 0)
    {
        if (sampled.size() < 2)
            return;

        std::uniform_int_distribution<size_t> distribution(
            0,
            sampled.size() - 1);
        std::random_device rd;
        std::mt19937 generator(rd());
        const size_t first = distribution(generator);
        size_t second = distribution(generator);
        while (second == first)
            second = distribution(generator);

        addNode(sampled[first]);
        addNode(sampled[second]);
    }

    for (const auto &point :
         sampled)
    {
        auto result =
            winnerSearch(point);

        //
        // Case (a)
        //

        if (result.d1 >= vigilance)
        {
            addNode(point);
            continue;
        }

        //
        // Case (b)
        //

        updateWinner(
            point,
            result.s1);

        if (result.d2 < vigilance)
            updateNeighbors(point, result.s1);

        for (int neighbor : map.neighbors(result.s1))
        {
            map.graph[result.s1][neighbor]++;
            map.graph[neighbor][result.s1]++;
        }

        estimateNormal(result.s1);
        estimateTraversability(result.s1);
        detectContour(result.s1);

        if (result.d2 < vigilance)
            updateEdge(result.s1, result.s2);

        removeOldEdges(result.s1, computeGmax(result.s1));
    }
}

// ==========================================================
// computeGthr()
// ==========================================================

float ATCDT::computeGthr(
    const std::vector<float> &ages) const
{
    if (ages.empty())
    {
        return std::numeric_limits<float>::infinity();
    }

    auto sorted = ages;
    std::sort(sorted.begin(), sorted.end());

    auto percentile = [&](float p)
    {
        float index = p * (sorted.size() - 1);

        size_t lower =
            static_cast<size_t>(std::floor(index));

        size_t upper =
            static_cast<size_t>(std::ceil(index));

        if (lower == upper)
        {
            return sorted[lower];
        }

        float weight =
            index - lower;

        return sorted[lower] * (1.0f - weight)
             + sorted[upper] * weight;
    };

    float q1 = percentile(0.25f);
    float q3 = percentile(0.75f);

    float iqr = q3 - q1;

    return q3 + iqr;
}

// ==========================================================
// computeGmax()
// ==========================================================

float ATCDT::computeGmax(
    int s1) const
{
    std::vector<float> current;
    const auto graph_it = map.graph.find(s1);
    if (graph_it != map.graph.end())
    {
        for (const auto &[neighbor, age] : graph_it->second)
        {
            (void)neighbor;
            current.push_back(static_cast<float>(age));
        }
    }

    float gthr =
        computeGthr(current);

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

void ATCDT::estimateNormal(int node)
{
    const auto node_neighbors = map.neighbors(node);
    if (node_neighbors.size() < 3)
        return;

    const Eigen::Vector3f center = map.positions[node];
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();

    for (int neighbor : node_neighbors)
    {
        const Eigen::Vector3f diff = map.positions[neighbor] - center;
        covariance += diff * diff.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    if (solver.info() == Eigen::Success)
        map.normals[node] = solver.eigenvectors().col(0);
}

void ATCDT::estimateTraversability(int node)
{
    const Eigen::Vector3f normal = map.normals[node];
    if (normal.norm() < 1e-6f)
        return;

    float cosine = std::clamp(
        std::abs(normal.normalized().dot(gravity)),
        0.0f,
        1.0f);
    map.slope_angles[node] = std::acos(cosine);
    map.traversable[node] = map.slope_angles[node] < max_slope;
}

void ATCDT::detectContour(int node)
{
    std::vector<int> node_neighbors = map.neighbors(node);
    if (exclude_untraversable_from_contour)
    {
        node_neighbors.erase(
            std::remove_if(
                node_neighbors.begin(),
                node_neighbors.end(),
                [this](int neighbor)
                {
                    return map.normals[neighbor].norm() < 1e-6f
                        || !map.traversable[neighbor];
                }),
            node_neighbors.end());
    }

    if (node_neighbors.size() < 2)
    {
        map.contour[node] = true;
        return;
    }

    Eigen::Vector3f reference(1.0f, 0.0f, 0.0f);
    if (std::abs(reference.dot(gravity)) > 0.9f)
        reference = Eigen::Vector3f(0.0f, 1.0f, 0.0f);

    Eigen::Vector3f u = gravity.cross(reference).normalized();
    Eigen::Vector3f v = gravity.cross(u);
    std::vector<float> angles;
    angles.reserve(node_neighbors.size());

    for (int neighbor : node_neighbors)
    {
        Eigen::Vector3f projected =
            map.positions[neighbor] - map.positions[node];
        projected -= projected.dot(gravity) * gravity;
        angles.push_back(std::atan2(projected.dot(v), projected.dot(u)));
    }

    std::sort(angles.begin(), angles.end());
    float largest_gap =
        (angles.front() + 2.0f * static_cast<float>(M_PI))
        - angles.back();
    for (std::size_t i = 1; i < angles.size(); ++i)
        largest_gap = std::max(largest_gap, angles[i] - angles[i - 1]);

    map.contour[node] = largest_gap > contour_gap;
}