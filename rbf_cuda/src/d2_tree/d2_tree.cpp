#include "d2_tree/d2_tree.h"

namespace d2_tree
{
    std::unique_ptr<KDNode> buildKDTree(std::vector<Point2D>& points, int depth)
    {
        if (points.empty()) return nullptr;

        int axis = depth % 2; // 0 for x, 1 for y
        auto cmp = [axis](const Point2D& a, const Point2D& b) {
            return (axis == 0) ? (a.x < b.x) : (a.y < b.y);
        };

        std::sort(points.begin(), points.end(), cmp);
        size_t mid = points.size() / 2;

        auto node = std::make_unique<KDNode>(points[mid]);
        std::vector<Point2D> leftPoints(points.begin(), points.begin() + mid);
        std::vector<Point2D> rightPoints(points.begin() + mid + 1, points.end());

        node->left = buildKDTree(leftPoints, depth + 1);
        node->right = buildKDTree(rightPoints, depth + 1);

        return node;
    }

    double distance(const Point2D& p1, const Point2D& p2)
    {
        return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    void nearestNeighborSearch(const std::unique_ptr<KDNode>& root, const Point2D& target, 
                               int depth, Point2D& best_point, double& best_dist)
    {
        if (!root) return;

        // 计算当前节点到目标的距离
        double current_dist = distance(root->point, target);
        if (current_dist < best_dist) {
            best_dist = current_dist;
            best_point = root->point;
        }

        // 确定划分轴和差值
        int axis = depth % 2;
        double diff = (axis == 0) ? (target.x - root->point.x) : (target.y - root->point.y);

        // 优先搜索更可能包含最近点的子树
        const auto& near_subtree = (diff <= 0) ? root->left : root->right;
        const auto& far_subtree = (diff <= 0) ? root->right : root->left;

        nearestNeighborSearch(near_subtree, target, depth + 1, best_point, best_dist);

        // 检查是否需要搜索另一侧的子树
        if (std::abs(diff) < best_dist) {
            nearestNeighborSearch(far_subtree, target, depth + 1, best_point, best_dist);
        }
    }

    Point2D nearestNeighbor(const std::unique_ptr<KDNode>& root, const Point2D& target)
    {
        Point2D best;
        double bestDist = std::numeric_limits<double>::max();
        nearestNeighborSearch(root, target, 0, best, bestDist);
        return best;
    }

    bool radiusSearch(const std::unique_ptr<KDNode>& root, const Point2D& target, double radius, int depth)
    {
        if (!root) return false;

        // 检查当前节点是否在半径范围内
        if (distance(root->point, target) <= radius) {
            return true; // 找到点，返回成功
        }

        // 确定当前划分轴
        int axis = depth % 2;
        double diff = (axis == 0) ? (target.x - root->point.x) : (target.y - root->point.y);

        // 决定搜索顺序
        if (diff <= 0) {
            if (radiusSearch(root->left, target, radius, depth + 1)) {
                return true; // 左子树找到点
            }
            if (std::abs(diff) <= radius && radiusSearch(root->right, target, radius, depth + 1)) {
                return true; // 右子树找到点
            }
        } else {
            if (radiusSearch(root->right, target, radius, depth + 1)) {
                return true; // 右子树找到点
            }
            if (std::abs(diff) <= radius && radiusSearch(root->left, target, radius, depth + 1)) {
                return true; // 左子树找到点
            }
        }

        return false; // 未找到点
    }

};

