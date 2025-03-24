#if 0
#include <iostream>
#include <vector>
#include <limits>
#include <Eigen/Dense>

// -------------------------------------------------------------------
// AABB Structure
// -------------------------------------------------------------------
struct AABB {
    Eigen::Vector3d min;
    Eigen::Vector3d max;

    AABB() {
        min = Eigen::Vector3d(std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max());
        max = Eigen::Vector3d(-std::numeric_limits<double>::max(),
            -std::numeric_limits<double>::max(),
            -std::numeric_limits<double>::max());
    }

    // Expand this AABB to include a point.
    void expand(const Eigen::Vector3d& point) {
        min = min.cwiseMin(point);
        max = max.cwiseMax(point);
    }

    // Expand to include another AABB.
    void expand(const AABB& other) {
        expand(other.min);
        expand(other.max);
    }

    // Check overlap between two AABBs.
    bool overlaps(const AABB& other) const {
        return (min.x() <= other.max.x() && max.x() >= other.min.x() &&
            min.y() <= other.max.y() && max.y() >= other.min.y() &&
            min.z() <= other.max.z() && max.z() >= other.min.z());
    }
};

// -------------------------------------------------------------------
// Dynamic AABB Tree Node
// -------------------------------------------------------------------
struct DynamicTreeNode {
    AABB box;
    int objectIndex; // if leaf, this is the object index; internal nodes have objectIndex == -1.
    DynamicTreeNode* left;
    DynamicTreeNode* right;
    DynamicTreeNode* parent;

    DynamicTreeNode(int idx)
        : objectIndex(idx), left(nullptr), right(nullptr), parent(nullptr) {}

    bool isLeaf() const {
        return left == nullptr && right == nullptr;
    }
};

// -------------------------------------------------------------------
// Dynamic AABB Tree for Broad-Phase Collision Detection
// -------------------------------------------------------------------
class DynamicAABBTree {
public:
    DynamicAABBTree() : root(nullptr) {}

    // Insert a leaf corresponding to an object and its AABB.
    DynamicTreeNode* insert(const AABB& box, int objectIndex) {
        DynamicTreeNode* node = new DynamicTreeNode(objectIndex);
        node->box = box;
        insertLeaf(node);
        return node;
    }

    // Query the tree for nodes whose AABB overlaps with queryBox.
    void query(const AABB& queryBox, std::vector<int>& results) const {
        queryRecursive(root, queryBox, results);
    }

    // Update a leaf's AABB (simplified: remove and reinsert).
    void update(DynamicTreeNode* node, const AABB& newBox) {
        removeLeaf(node);
        node->box = newBox;
        insertLeaf(node);
    }

    ~DynamicAABBTree() {
        destroyRecursive(root);
    }

private:
    DynamicTreeNode* root;

    void destroyRecursive(DynamicTreeNode* node) {
        if (!node) return;
        destroyRecursive(node->left);
        destroyRecursive(node->right);
        delete node;
    }

    // Helper to compute the volume of an AABB.
    double volume(const AABB& box) const {
        Eigen::Vector3d diff = box.max - box.min;
        return diff.x() * diff.y() * diff.z();
    }

    // Insert a leaf into the tree.
    void insertLeaf(DynamicTreeNode* leaf) {
        if (root == nullptr) {
            root = leaf;
            return;
        }

        // Find the best sibling for insertion by minimizing the increase in volume.
        DynamicTreeNode* sibling = root;
        while (!sibling->isLeaf()) {
            AABB combined = sibling->box;
            combined.expand(leaf->box);
            double cost = volume(combined) - volume(sibling->box);

            double leftCost = 0.0, rightCost = 0.0;
            if (sibling->left) {
                AABB leftCombined = sibling->left->box;
                leftCombined.expand(leaf->box);
                leftCost = volume(leftCombined) - volume(sibling->left->box);
            }
            if (sibling->right) {
                AABB rightCombined = sibling->right->box;
                rightCombined.expand(leaf->box);
                rightCost = volume(rightCombined) - volume(sibling->right->box);
            }

            // Descend into the child with lower cost.
            if (leftCost < rightCost)
                sibling = sibling->left;
            else
                sibling = sibling->right;
        }

        // Once a leaf is found, create a new parent.
        DynamicTreeNode* oldParent = sibling->parent;
        DynamicTreeNode* newParent = new DynamicTreeNode(-1);
        newParent->parent = oldParent;
        newParent->box = sibling->box;
        newParent->box.expand(leaf->box);
        newParent->left = sibling;
        newParent->right = leaf;
        sibling->parent = newParent;
        leaf->parent = newParent;

        if (oldParent == nullptr) {
            root = newParent;
        }
        else {
            if (oldParent->left == sibling)
                oldParent->left = newParent;
            else
                oldParent->right = newParent;
            // Update AABB up the tree.
            DynamicTreeNode* current = oldParent;
            while (current != nullptr) {
                AABB newBox;
                if (current->left)
                    newBox.expand(current->left->box);
                if (current->right)
                    newBox.expand(current->right->box);
                current->box = newBox;
                current = current->parent;
            }
        }
    }

    // Remove a leaf from the tree.
    void removeLeaf(DynamicTreeNode* leaf) {
        if (leaf == root) {
            root = nullptr;
            return;
        }
        DynamicTreeNode* parent = leaf->parent;
        DynamicTreeNode* grandParent = parent->parent;
        DynamicTreeNode* sibling = (parent->left == leaf) ? parent->right : parent->left;

        if (grandParent == nullptr) {
            root = sibling;
            sibling->parent = nullptr;
        }
        else {
            if (grandParent->left == parent)
                grandParent->left = sibling;
            else
                grandParent->right = sibling;
            sibling->parent = grandParent;
            // Update the ancestors' AABBs.
            DynamicTreeNode* current = grandParent;
            while (current != nullptr) {
                AABB newBox;
                if (current->left)
                    newBox.expand(current->left->box);
                if (current->right)
                    newBox.expand(current->right->box);
                current->box = newBox;
                current = current->parent;
            }
        }
        delete parent;
    }

    // Recursively query the tree.
    void queryRecursive(DynamicTreeNode* node, const AABB& queryBox, std::vector<int>& results) const {
        if (!node) return;
        if (!node->box.overlaps(queryBox))
            return;
        if (node->isLeaf())
            results.push_back(node->objectIndex);
        else {
            queryRecursive(node->left, queryBox, results);
            queryRecursive(node->right, queryBox, results);
        }
    }
};

// -------------------------------------------------------------------
// Example CCD Test Stubs (placeholders for actual CCD implementations)
// -------------------------------------------------------------------
bool CCD_Sphere_Sphere(int idxA, int idxB) {
    // Placeholder for fast sphere-sphere continuous collision detection.
    return true;
}
bool CCD_Ellipsoid_Ellipsoid(int idxA, int idxB) {
    // Placeholder for ellipsoid-ellipsoid CCD.
    return true;
}
bool CCD_OBB_OBB(int idxA, int idxB) {
    // Placeholder for oriented bounding box CCD.
    return true;
}
bool CCD_Convex_Convex(int idxA, int idxB) {
    // Placeholder for convex set CCD.
    return true;
}

// -------------------------------------------------------------------
// Object with Layered Bounding Volumes
// -------------------------------------------------------------------
struct Object {
    int id;
    // For simplicity, we assume each layer is stored as an AABB.
    // In a full system these would be derived from your actual geometry and transforms.
    AABB sphereAABB;    // Outer bounding volume (e.g., sphere approximated as AABB)
    AABB ellipsoidAABB; // Next layer (ellipsoid approximated as AABB)
    AABB obbAABB;       // Oriented bounding box layer
    AABB convexAABB;    // Actual convex shape AABB (tight-fitting)
};

// -------------------------------------------------------------------
// Main Demonstration
// -------------------------------------------------------------------
int test() {
    // Create a few objects.
    std::vector<Object> objects;

    // Object 0
    Object obj0;
    obj0.id = 0;
    // For demonstration, we set the same AABB for all layers.
    obj0.sphereAABB.min = Eigen::Vector3d(-1, -1, -1);
    obj0.sphereAABB.max = Eigen::Vector3d(1, 1, 1);
    obj0.ellipsoidAABB = obj0.sphereAABB;
    obj0.obbAABB = obj0.sphereAABB;
    obj0.convexAABB = obj0.sphereAABB;
    objects.push_back(obj0);

    // Object 1
    Object obj1;
    obj1.id = 1;
    obj1.sphereAABB.min = Eigen::Vector3d(0.5, 0.5, 0.5);
    obj1.sphereAABB.max = Eigen::Vector3d(2.5, 2.5, 2.5);
    obj1.ellipsoidAABB = obj1.sphereAABB;
    obj1.obbAABB = obj1.sphereAABB;
    obj1.convexAABB = obj1.sphereAABB;
    objects.push_back(obj1);

    // Build a dynamic AABB tree using the outer bounding volumes (spheres).
    DynamicAABBTree tree;
    std::vector<DynamicTreeNode*> nodes;
    for (size_t i = 0; i < objects.size(); i++) {
        DynamicTreeNode* node = tree.insert(objects[i].sphereAABB, objects[i].id);
        nodes.push_back(node);
    }

    // In a dynamic simulation, objects move/rotate and you would update each node’s AABB.
    // Here, we perform a query on each object’s sphereAABB to find potential collisions.
    for (size_t i = 0; i < objects.size(); i++) {
        std::vector<int> potentialCollisions;
        tree.query(objects[i].sphereAABB, potentialCollisions);
        std::cout << "Object " << objects[i].id << " potential collisions: ";
        for (int idx : potentialCollisions) {
            if (idx == objects[i].id)
                continue; // Skip self

            std::cout << idx << " ";
            // Cascade layered CCD tests:
            if (CCD_Sphere_Sphere(objects[i].id, idx)) {
                if (CCD_Ellipsoid_Ellipsoid(objects[i].id, idx)) {
                    if (CCD_OBB_OBB(objects[i].id, idx)) {
                        if (CCD_Convex_Convex(objects[i].id, idx)) {
                            std::cout << "[Collision confirmed with " << idx << "] ";
                        }
                    }
                }
            }
        }
        std::cout << "\n";
    }

    return 0;
}
#endif