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

        std::vector<int8_t> traversability;

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
        float vigilance = 0.5f,
        int lambda_points = 4000);

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

    void updateExistingNode(
        const Eigen::Vector3f &point,
        int winner);

    void updateEdge(
        int s1,
        int s2);

    void removeOldEdges(
        float gmax);

    float computeGthr() const;

    float computeGmax() const;
};