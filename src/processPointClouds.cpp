// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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

            if (point[cd] < ((*node)->point[cd]))
                insertHelper(&((*node)->left),depth+1, point, id);
            else
                insertHelper(&((*node)->right),depth+1, point, id);
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
            if( (node->point[0]>=(target[0]-distanceTol)&&node->point[0]<=(target[0]+distanceTol))
                && (node->point[1]>=(target[1]-distanceTol)&&node->point[1]<=(target[1]+distanceTol)))
            {
                float distance = std::sqrt((node->point[0]-target[0])*(node->point[0]-target[0])+(node->point[1]-target[1])*(node->point[1]-target[1]));
                if (distance <= distanceTol)
                    ids.push_back(node->id);
            }

            // check accross boundary
            if((target[depth%2]-distanceTol)<node->point[depth%2])
                searchHelper(target, node->left, depth+1, distanceTol, ids);
            if((target[depth%2]+distanceTol)>node->point[depth%2])
                searchHelper(target, node->right, depth+1, distanceTol, ids);

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
void clusterHelper2(int indice, const std::vector<PointT> points, std::vector<int>& cluster, std::vector<bool>& processed, myKdTree<PointT>* tree, float distanceTol)
{
    processed[indice] = true;
    cluster.push_back(indice);

    std::vector<int> nearest = tree->search(points[indice],distanceTol);

    for(int id: nearest)
    {
        if(!processed[id])
            clusterHelper2(id, points, cluster, processed, tree, distanceTol);
    }
};

template<typename PointT>
std::vector<std::vector<PointT>> myEuclideanCluster(const std::vector<PointT>& points, myKdTree<PointT>* tree, float distanceTol)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<PointT>> clusters;

    std::vector<bool> processed(points.size(), false);

    int i = 0;
    while (i < points.size()) {
        if (processed[i]) {
            i++;
            continue;
        }
        std::vector<int> cluster;
        clusterHelper2(i, points, cluster, processed, tree, distanceTol);
        clusters.push_back(cluster);
        i++;
    }
    return clusters;
};

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

	// Create the voxel grid box: downsample the dataset using a leaf size of .2m
	pcl::VoxelGrid<PointT> vg;
	typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);

	vg.setInputCloud(cloud);
	vg.setLeafSize(filterRes, filterRes, filterRes);
	vg.filter(*cloudFiltered);

	typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

	pcl::CropBox<PointT> region(true);  // true because we are dealing with points inside the cropbox
	region.setMin(minPoint);
	region.setMax(maxPoint);
	region.setInputCloud(cloudFiltered);
	region.filter(*cloudRegion);

	std::vector<int> indices;

	//removing the roof points
	pcl::CropBox<PointT> roof(true);
	roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));	// region defined to remove the roof points
	roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
	roof.setInputCloud(cloudRegion);
	roof.filter(indices);

	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
	for (int point:indices)
		inliers->indices.push_back(point);
	
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloudRegion);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    for (int index:inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);
    
    pcl::ExtractIndices<PointT> extract;
    //extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
pcl::PointIndices::Ptr ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
    pcl::PointIndices::Ptr inliers2 {new pcl::PointIndices};

	// own Ransac Implementation
	while(maxIterations--){
		std::unordered_set<int> inliers;
		while(inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));

		float x1, x2, x3, y1, y2, y3, z1, z2, z3;

		auto itr = inliers.begin(); //begin gives a pointer to the start of inliers
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float vec1[3] = {x2-x1,y2-y1,z2-z1};
		float vec2[3] = {x3-x1,y3-y1,z3-z1};

		float ui = (vec1[1]*vec2[2] - vec1[2]*vec2[1]);
		float uj = (vec1[2]*vec2[0] - vec1[0]*vec2[2]);
		float uk = (vec1[0]*vec2[1] - vec1[1]*vec2[0]);
		
		float a = ui;
		float b = uj;
		float c = uk;
		float d = -(ui*x3 + uj*y3 + uk*z3);

		for (int index=0; index < cloud->points.size(); index++){

			if (inliers.count(index) > 0)
				continue;

			PointT point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			float dist = fabs(a*x4 + b*y4 + c*z4 + d)/sqrt(a*a+b*b+c*c);

			if (dist <= distanceTol)
            {
                inliers.insert(index);
            }

		}
        // storing the inliers with bigger results
		if (inliers.size() > inliersResult.size()){
			inliersResult = inliers;
		}
	}
    // translating the indices from unordered_set into pcl::PointIndices::Ptr
    for (auto idx:inliersResult)
    {
        inliers2->indices.push_back(idx);
    }

	return inliers2;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    //pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    
    // obtaining the indices from the surface
	pcl::PointIndices::Ptr inliers = Ransac(cloud, 35, 0.2);

	/*
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);*/
	

    if(inliers->indices.size() == 0){
        std::cout << "Could not estimate a planar model for given data set" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

	std::vector<pcl::PointIndices> clusterIndices;


	// clusters is the vector of point clouds
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

	//Creating the Kd-Tree object for the search method extraction
	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);
	
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(clusterTolerance);
	ec.setMinClusterSize(minSize);
	ec.setMaxClusterSize(maxSize);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices: clusterIndices){
		typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

		for(int index: getIndices.indices)
			cloudCluster->points.push_back(cloud->points[index]);
		
		cloudCluster->width = cloudCluster->points.size ();
		cloudCluster->height = 1;
		cloudCluster->is_dense = true;
		// clusters is the vector of point clouds
		clusters.push_back(cloudCluster);

	}
    /*
    // own cluster
    myKdTree<PointT>* tree = new myKdTree<PointT>;

    for (int idx=0; idx < cloud->points.size(); idx++)
        tree->insert(cloud->points[idx],idx);

    std::vector<std::vector<PointT>> clustersIdx = myEuclideanCluster(cloud->points, tree, 3.0);
    // vector with clusters
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    for(int idx=0; idx < clustersIdx.size(); idx++)
    {
        typename pcl::PointCloud<PointT>::Ptr small_cloud;
        for(PointT point:clustersIdx[idx])
        {
            small_cloud->points.insert(point);
        }
        clusters[idx] = small_cloud;
    }*/

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

/*
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    //pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0){
        std::cout << "Could not estimate a planar model for given data set" << std::endl;
    }
    
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    
    //std::unordered_set<int> inliers = Ransac(cloud, maxIterations, distanceThreshold);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}
*/