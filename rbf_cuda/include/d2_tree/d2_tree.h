#ifndef D2_TREE_H
#define D2_TREE_H

#include <vector>
#include <algorithm>
#include <cmath>
#include <memory>
#include <limits>

namespace d2_tree
{
    struct Point2D {
        double x, y;
        Point2D(double x = 0, double y = 0) : x(x), y(y) {}
    };

    struct KDNode {
        Point2D point;
        std::unique_ptr<KDNode> left;
        std::unique_ptr<KDNode> right;

        KDNode(Point2D p) : point(p), left(nullptr), right(nullptr) {}
    };

    std::unique_ptr<KDNode> buildKDTree(std::vector<Point2D>& points, int depth = 0);

    double distance(const Point2D& p1, const Point2D& p2);

    void nearestNeighborSearch(const std::unique_ptr<KDNode>& root, const Point2D& target, 
                               int depth, Point2D& best_point, double& best_dist);

    Point2D nearestNeighbor(const std::unique_ptr<KDNode>& root, const Point2D& target);

    bool radiusSearch(const std::unique_ptr<KDNode>& root, const Point2D& target, double radius, int depth = 0);


} // namespace d2_tree


#endif