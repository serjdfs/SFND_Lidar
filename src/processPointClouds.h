// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>

// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
    PointT point;
    int id;
    Node* left;
    Node* right;

    Node(PointT arr, int setId)
            :	point(arr), id(setId), left(NULL), right(NULL)
    {}
};


template<typename PointT>
struct myKdTree
{
    Node<PointT>* root;

    myKdTree()
            : root(NULL)
    {}

    void insertHelper(Node<PointT>** node, uint depth, PointT point, int id)
    {
        //Tree is empty
        if (*node == NULL)
            *node = new Node<PointT>(point, id);
        else
        {
            //calculate current dimension; we get 0s and 1s
            uint cd = depth % 2;
            if(cd == 0)
            {
                if (point.x < ((*node)->point.x))
                    insertHelper(&((*node)->left),depth+1, point, id);
                else
                    insertHelper(&((*node)->right),depth+1, point, id);
            } else{
                if (point.y < ((*node)->point.y))
                    insertHelper(&((*node)->left),depth+1, point, id);
                else
                    insertHelper(&((*node)->right),depth+1, point, id);
            }

        }

    }

    void insert(PointT point, int id)
    {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root

        insertHelper(&root, 0, point, id);
    }

    void searchHelper(PointT target, Node<PointT>* node, int depth, float distanceTol, std::vector<int>& ids)
    {
        if(node!=NULL)
        {
            if( (node->point.x>=(target.x-distanceTol)&&node->point.x<=(target.x+distanceTol))
                && (node->point.y>=(target.y-distanceTol)&&node->point.y<=(target.y+distanceTol)))
            {
                float distance = std::sqrt((node->point.x-target.x)*(node->point.x-target.x)+(node->point.y-target.y)*(node->point.y-target.y));
                if (distance <= distanceTol)
                    ids.push_back(node->id);
            }

            if(depth%2 == 0)
            {
                // check accross boundary
                if((target.x-distanceTol)<node->point.x)
                    searchHelper(target, node->left, depth+1, distanceTol, ids);
                if((target.x+distanceTol)>node->point.x)
                    searchHelper(target, node->right, depth+1, distanceTol, ids);
            }else{
                // check accross boundary
                if((target.y-distanceTol)<node->point.y)
                    searchHelper(target, node->left, depth+1, distanceTol, ids);
                if((target.y+distanceTol)>node->point.y)
                    searchHelper(target, node->right, depth+1, distanceTol, ids);
            }


        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(PointT target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);

        return ids;
    }
};


template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);
	//std::unordered_set<int>
    pcl::PointIndices::Ptr Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    void clusterHelper2(int indice, std::vector<PointT> points, std::vector<int>& cluster, std::vector<bool>& processed, myKdTree<PointT>* tree, float distanceTol);

    std::vector<std::vector<PointT>> myEuclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, myKdTree<PointT>* tree, float distanceTol);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};

#endif /* PROCESSPOINTCLOUDS_H_ */