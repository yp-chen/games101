#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    //得到所有物体的包围盒
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        std::vector<Object*>::iterator middling;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        //选择最大维度来进行划分，排序
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        if (splitMethod == SplitMethod::NAIVE) {
            middling = objects.begin() + (objects.size() / 2);
        }
        else if (splitMethod == SplitMethod::SAH) {
            float totalSurfaceArea = centroidBounds.SurfaceArea();
            int bucketSize = 10;//桶的数量
            int optimalSplitIndex = 0;
            float minCost = std::numeric_limits<float>::infinity(); //最小花费
            float trav_cost = 0.125;
            for (int i=0;i<bucketSize;++i) {
                auto beginning = objects.begin();
                auto middling = objects.begin() + (objects.size() * i / bucketSize);
                auto ending = objects.end();
                auto leftshapes = std::vector<Object*>(beginning, middling);
                auto rightshapes = std::vector<Object*>(middling, ending);
                Bounds3 leftbounds,rightbounds;
                for (int i = 0; i < leftshapes.size(); ++i){
                    leftbounds =Union(leftbounds, leftshapes[i]->getBounds().Centroid());
                }
                for (int i = 0; i < rightshapes.size(); ++i){
                    rightbounds = Union(rightbounds, rightshapes[i]->getBounds().Centroid());
                }
                double SA = leftbounds.SurfaceArea();
                double SB = rightbounds.SurfaceArea();
                float cost = trav_cost + (SA * leftshapes.size() + SB * rightshapes.size()) / totalSurfaceArea;
                if (cost < minCost){
                    minCost = cost;
                    optimalSplitIndex = i;
                }
            }
            middling = objects.begin() + (objects.size() * optimalSplitIndex / bucketSize);
        }

        auto beginning = objects.begin();
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection intersection;
    std::array<int, 3> dirIsNeg;
    dirIsNeg[0] = (ray.direction.x < 0);
    dirIsNeg[1] = (ray.direction.y < 0);
    dirIsNeg[2] = (ray.direction.z < 0);
    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg)) {
        return intersection;
    }
    if (node->left == nullptr && node->right == nullptr) {
        intersection = node->object->getIntersection(ray);
        return intersection;
    }
    Intersection leftIntersection = getIntersection(node->left, ray);
    Intersection rightIntersection = getIntersection(node->right, ray);
    return leftIntersection.distance < rightIntersection.distance ? leftIntersection : rightIntersection;
}