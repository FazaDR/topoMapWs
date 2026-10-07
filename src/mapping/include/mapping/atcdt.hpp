#pragma once

#include <Eigen/Core>

#include <unordered_map>
#include <vector>
#include <limits>
#include <cstdint>
#include <utility>

class ATCDT
{
public:

    class TopologicalMap
    {
    public:

        //---------------------------------------
        // Node set V
        //---------------------------------------

        int node_count;

        //---------------------------------------
        // Reference vectors
        //---------------------------------------

        std::vector<Eigen::Vector3f> positions;

        std::vector<Eigen::Vector3f> normals;

        std::vector<float> slope_angles;

        std::vector<bool> traversable;

        std::vector<bool> contour;

        //---------------------------------------
        // Edge set
        //---------------------------------------

        std::unordered_map<
            int,
            std::unordered_map<int,int>
        > graph;

        //---------------------------------------
        // ctor
        //---------------------------------------

        TopologicalMap();

        //---------------------------------------
        // Node operations
        //---------------------------------------

        int addNode(
            const Eigen::Vector3f &position);

        //---------------------------------------
        // Edge operations
        //---------------------------------------

        bool hasEdge(
            int a,
            int b) const;

        void addEdge(
            int a,
            int b);

        void removeEdge(
            int a,
            int b);

        std::vector<int> neighbors(
            int node) const;

        //---------------------------------------
        // Utilities
        //---------------------------------------

        std::vector<std::pair<int,int>> edges() const;

        std::vector<float> edgeAges() const;

        int numEdges() const;
    };

public:

    ATCDT(
        float vigilance = 0.9f,
        int lambda_points = 4000,
        float max_slope = 30.0f * 3.14159265358979323846f / 180.0f,
        float contour_gap = 3.14159265358979323846f,
        const Eigen::Vector3f &gravity = Eigen::Vector3f(0.0f, -1.0f, 0.0f),
        bool exclude_untraversable_from_contour = true);

    //---------------------------------------
    // Parameters
    //---------------------------------------

    float vigilance;

    int lambda_points;

    //---------------------------------------
    // State
    //---------------------------------------

    std::vector<float> deleted_edge_ages;

    std::vector<int> winner_count;

    TopologicalMap map;

    float max_slope;
    float contour_gap;
    Eigen::Vector3f gravity;
    bool exclude_untraversable_from_contour;

    //---------------------------------------
    // Main algorithm
    //---------------------------------------

    void processFrame(
        const std::vector<Eigen::Vector3f> &point_cloud);

private:

    //---------------------------------------
    // Helpers
    //---------------------------------------

    std::vector<Eigen::Vector3f> samplePoints(
        const std::vector<Eigen::Vector3f> &points) const;

    int addNode(
        const Eigen::Vector3f &point);

    struct WinnerResult
    {
        int s1 = -1;
        int s2 = -1;

        float d1 = std::numeric_limits<float>::infinity();
        float d2 = std::numeric_limits<float>::infinity();
    };

    WinnerResult winnerSearch(
        const Eigen::Vector3f &point) const;

    void updateWinner(
        const Eigen::Vector3f &point,
        int winner);

    void updateNeighbors(
        const Eigen::Vector3f &point,
        int winner);

    void updateEdge(
        int s1,
        int s2);

    void removeOldEdges(
        int s1,
        float gmax);

    float computeGthr(
        const std::vector<float> &ages) const;

    float computeGmax(
        int s1) const;

    void estimateNormal(int node);
    void estimateTraversability(int node);
    void detectContour(int node);
};