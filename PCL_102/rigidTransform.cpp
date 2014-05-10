#include "rigidTransform.h"

#include "addColor.h"
#include "visualizePCD.h"


using namespace pcl;
using namespace pcl::registration;



void rigidTranform(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,const  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, Eigen::Matrix4f transform, string filename_base)
{
//	correspondences_demo(target_cloud, input_cloud, filename_base.c_str());

}


void
downsample (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float leaf_size,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &downsampled_out)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
  vox_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
  vox_grid.setInputCloud (points);
  vox_grid.filter (*downsampled_out);
}

void
compute_surface_normals (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float normal_radius,
                         pcl::PointCloud<pcl::Normal>::Ptr &normals_out)
{
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;

  // Use a FLANN-based KdTree to perform neighborhood searches
  //norm_est.setSearchMethod (pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr (new pcl::KdTreeFLANN<pcl::PointXYZRGB>));
  norm_est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));


  // Specify the size of the local neighborhood to use when computing the surface normals
  norm_est.setRadiusSearch (normal_radius);

  // Set the input points
  norm_est.setInputCloud (points);

  // Estimate the surface normals and store the result in "normals_out"
  norm_est.compute (*normals_out);
}

void
compute_PFH_features (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                      pcl::PointCloud<pcl::Normal>::Ptr &normals,
                      float feature_radius,
                      pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out)
{
  // Create a PFHEstimation object
  pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh_est;

  // Set it to use a FLANN-based KdTree to perform its neighborhood searches
  pfh_est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));

  // Specify the radius of the PFH feature
  pfh_est.setRadiusSearch (feature_radius);

  // Set the input points and surface normals
  pfh_est.setInputCloud (points);
  pfh_est.setInputNormals (normals);

  // Compute the features
  pfh_est.compute (*descriptors_out);
}

void
detect_keypoints (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                  float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast,
                  pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_out)
{
  pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift_detect;

  // Use a FLANN-based KdTree to perform neighborhood searches
  sift_detect.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));

  // Set the detection parameters
  sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
  sift_detect.setMinimumContrast (min_contrast);

  // Set the input
  sift_detect.setInputCloud (points);

  // Detect the keypoints and store them in "keypoints_out"
  sift_detect.compute (*keypoints_out);
}

void
compute_PFH_features_at_keypoints (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                                   pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                   pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints, float feature_radius,
                                   pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out)
{
  // Create a PFHEstimation object
  pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh_est;

  // Set it to use a FLANN-based KdTree to perform its neighborhood searches
  pfh_est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));

  // Specify the radius of the PFH feature
  pfh_est.setRadiusSearch (feature_radius);

  /* This is a little bit messy: since our keypoint detection returns PointWithScale points, but we want to
   * use them as an input to our PFH estimation, which expects clouds of PointXYZRGB points.  To get around this,
   * we'll use copyPointCloud to convert "keypoints" (a cloud of type PointCloud<PointWithScale>) to
   * "keypoints_xyzrgb" (a cloud of type PointCloud<PointXYZRGB>).  Note that the original cloud doesn't have any RGB
   * values, so when we copy from PointWithScale to PointXYZRGB, the new r,g,b fields will all be zero.
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud (*keypoints, *keypoints_xyzrgb);

  // Use all of the points for analyzing the local structure of the cloud
  pfh_est.setSearchSurface (points);
  pfh_est.setInputNormals (normals);

  // But only compute features at the keypoints
  pfh_est.setInputCloud (keypoints_xyzrgb);

  // Compute the features
  pfh_est.compute (*descriptors_out);
}

void
find_feature_correspondences (pcl::PointCloud<pcl::PFHSignature125>::Ptr &source_descriptors,
                              pcl::PointCloud<pcl::PFHSignature125>::Ptr &target_descriptors,
                              std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out)
{
  // Resize the output vector
  correspondences_out.resize (source_descriptors->size ());
  correspondence_scores_out.resize (source_descriptors->size ());

  // Use a KdTree to search for the nearest matches in feature space
  pcl::search::KdTree<pcl::PFHSignature125> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target_descriptors);

  // Find the index of the best match for each keypoint, and store it in "correspondences_out"
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (size_t i = 0; i < source_descriptors->size (); ++i)
  {
    descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
    correspondences_out[i] = k_indices[0];
    correspondence_scores_out[i] = k_squared_distances[0];
  }
}

void visualize_normals (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,
                        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal_points,
                        const pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  // Add the points and normals to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points, "points");
  viz.addPointCloud (normal_points, "normal_points");

  viz.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (normal_points, normals, 1, 0.01, "normals");

  // Give control over to the visualizer
  viz.spin ();
}

void visualize_keypoints (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,
                          const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints)
{
  // Add the points to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points, "points");

  // Draw each keypoint as a sphere
  for (size_t i = 0; i < keypoints->size (); ++i)
  {
    // Get the point data
    const pcl::PointWithScale & p = keypoints->points[i];

    // Pick the radius of the sphere *
    float r = 2 * p.scale;
    // * Note: the scale is given as the standard deviation of a Gaussian blur, so a
    //   radius of 2*p.scale is a good illustration of the extent of the keypoint

    // Generate a unique string for each sphere
    std::stringstream ss ("keypoint");
    ss << i;

    // Add a sphere at the keypoint
    viz.addSphere (p, 2*p.scale, 1.0, 0.0, 0.0, ss.str ());
  }

  // Give control over to the visualizer
  viz.spin ();
}

void visualize_correspondences (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1,
                                const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1,
                                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2,
                                const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2,
                                const std::vector<int> &correspondences,
                                const std::vector<float> &correspondence_scores)
{
  // We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
  // by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points

  // Create some new point clouds to hold our transformed data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_left (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_left (new pcl::PointCloud<pcl::PointWithScale>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_right (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_right (new pcl::PointCloud<pcl::PointWithScale>);

  // Shift the first clouds' points to the left
  //const Eigen::Vector3f translate (0.0, 0.0, 0.3);
  const Eigen::Vector3f translate (0.4, 0.0, 0.0);
  const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
  pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
  pcl::transformPointCloud (*keypoints1, *keypoints_left, -translate, no_rotation);

  // Shift the second clouds' points to the right
  pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
  pcl::transformPointCloud (*keypoints2, *keypoints_right, translate, no_rotation);

  // Add the clouds to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points_left, "points_left");
  viz.addPointCloud (points_right, "points_right");

  // Compute the median correspondence score
  std::vector<float> temp (correspondence_scores);
  std::sort (temp.begin (), temp.end ());
  float median_score = temp[temp.size ()/2];

  // Draw lines between the best corresponding points
  for (size_t i = 0; i < keypoints_left->size (); ++i)
  {
    if (correspondence_scores[i] > median_score)
    {
      continue; // Don't draw weak correspondences
    }

    // Get the pair of points
    const pcl::PointWithScale & p_left = keypoints_left->points[i];
    const pcl::PointWithScale & p_right = keypoints_right->points[correspondences[i]];

    // Generate a random (bright) color
    double r = (rand() % 100);
    double g = (rand() % 100);
    double b = (rand() % 100);
    double max_channel = std::max (r, std::max (g, b));
    r /= max_channel;
    g /= max_channel;
    b /= max_channel;

    // Generate a unique string for each line
    std::stringstream ss ("line");
    ss << i;

    // Draw the line
    viz.addLine (p_left, p_right, r, g, b, ss.str ());
  }

  // Give control over to the visualizer
  viz.spin ();
}

int normals_demo (const char * filename)
{
  // Create some new point clouds to hold our data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

  // Load a point cloud
  pcl::io::loadPCDFile (filename, *points);

  // Downsample the cloud
  const float voxel_grid_leaf_size = 0.01;
  downsample (points, voxel_grid_leaf_size, downsampled);

  // Compute surface normals
  const float normal_radius = 0.03;
  compute_surface_normals (downsampled, normal_radius, normals);

  // Visualize the points and normals
  visualize_normals (points, downsampled, normals);

  return (0);
}

int keypoints_demo (const char * filename)
{

  // Create some new point clouds to hold our data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints (new pcl::PointCloud<pcl::PointWithScale>);

  // Load a point cloud
  pcl::io::loadPCDFile (filename, *points);

  // Compute keypoints
  const float min_scale = 0.01;
  const int nr_octaves = 3;
  const int nr_octaves_per_scale = 3;
  const float min_contrast = 10.0;
  detect_keypoints (points, min_scale, nr_octaves, nr_octaves_per_scale, min_contrast, keypoints);

  // Visualize the point cloud and its keypoints
  visualize_keypoints (points, keypoints);

  return (0);
}

int correspondences_demo (pcl::PointCloud<pcl::PointXYZ>::Ptr pointstemp1, pcl::PointCloud<pcl::PointXYZ>::Ptr pointstemp2, const char * filename_base, Eigen::Matrix4f &transMat)
{
  // Create some new point clouds to hold our data
	
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr normals1 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1 (new pcl::PointCloud<pcl::PointWithScale>);
  pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors1 (new pcl::PointCloud<pcl::PFHSignature125>);

  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2 (new pcl::PointCloud<pcl::PointWithScale>);
  pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors2 (new pcl::PointCloud<pcl::PFHSignature125>);
  cout<<"loading file1"<<endl;
 
  addColor(pointstemp1, points1, 255, 0, 0);
  addColor(pointstemp2, points2, 255, 0, 0);
  
  // Downsample the cloud
  const float voxel_grid_leaf_size = 0.01;
  cout<<"downsampling file1"<<endl;
  //downsample (points1, voxel_grid_leaf_size, downsampled1);
  cout<<"downsampling file2"<<endl;
  //downsample (points2, voxel_grid_leaf_size, downsampled2);
  cout<<"downsampling done"<<endl;
  
  downsampled1 = points1;
  downsampled2 = points2;
  
  //visualizePCD(downsampled1);
  //visualizePCD(downsampled2);


  bool useSavedNormals = true;
   bool useSavedKeypoints = true;
  bool useSavedPFHfeatures = true;

  if(useSavedNormals)
  {
	   std::stringstream ss1, ss2;
	 ss1 << filename_base<<"1_normals.pcd";
	  pcl::io::loadPCDFile (ss1.str (), *normals1);
	  
	  ss2 << filename_base<<"2_normals.pcd";
	  pcl::io::loadPCDFile (ss2.str (), *normals2);
	 
  }
  else
  {
  // Compute surface normals
    const float normal_radius = 0.03;
  cout<<"compute surface normals1"<<endl;
  compute_surface_normals (downsampled1, normal_radius, normals1);
  cout<<"compute surface normals2"<<endl;
  compute_surface_normals (downsampled2, normal_radius, normals2);
  
  std::stringstream ss;
		ss << filename_base<<"1_normals.pcd";
		pcl::io::savePCDFileASCII (ss.str (), *normals1);
		std::stringstream sss;
		sss << filename_base<<"2_normals.pcd";
		pcl::io::savePCDFileASCII (sss.str (), *normals2);
	
 
  }
 

  cout<<"compute surface normals done"<<endl;
  // Compute keypoints

  if(useSavedKeypoints)
  {
	  std::stringstream ss1, ss2;
	 ss1 << filename_base<<"1_keypoints.pcd";
	  pcl::io::loadPCDFile (ss1.str (), *keypoints1);
	
	  ss2 << filename_base<<"2_keypoints.pcd";
	  pcl::io::loadPCDFile (ss2.str (), *keypoints2);

  }
  else
  {
  const float min_scale = 0.000038;
  const int nr_octaves = 3;
  const int nr_octaves_per_scale = 3;
  const float min_contrast = 0.0;
  cout<<"detect keypoints1"<<endl;
  detect_keypoints (points1, min_scale, nr_octaves, nr_octaves_per_scale, min_contrast, keypoints1);
  cout<<"detect keypoints2"<<endl;
  detect_keypoints (points2, min_scale, nr_octaves, nr_octaves_per_scale, min_contrast, keypoints2);
 
  std::stringstream ss;
		ss << filename_base<<"1_keypoints.pcd";
		pcl::io::savePCDFileASCII (ss.str (), *keypoints1);
		std::stringstream sss;
		sss << filename_base<<"2_keypoints.pcd";
		pcl::io::savePCDFileASCII (sss.str (), *keypoints2);
	

  }
  	 cout<<"detect keypoints done"<<endl;
  // Compute PFH features


 
  if(useSavedPFHfeatures)
  {
	   std::stringstream ss1, ss2;
	 ss1 << filename_base<<"1_pfh.pcd";
	  pcl::io::loadPCDFile (ss1.str (), *descriptors1);
	
	  ss2 << filename_base<<"2_pfh.pcd";
	  pcl::io::loadPCDFile (ss2.str (), *descriptors2);
	 


  }

  else
  {
	  const float feature_radius = 0.08;
	  cout<<"compute pfh features1"<<endl;
	  compute_PFH_features_at_keypoints (downsampled1, normals1, keypoints1, feature_radius, descriptors1);
	  cout<<"compute pfh features2"<<endl;
	  compute_PFH_features_at_keypoints (downsampled2, normals2, keypoints2, feature_radius, descriptors2);
	  
	  stringstream ss;
	  ss << filename_base<<"1_pfh.pcd";
		pcl::io::savePCDFileASCII (ss.str (), *descriptors1);
		std::stringstream sss;
		sss << filename_base<<"2_pfh.pcd";
		pcl::io::savePCDFileASCII (sss.str (), *descriptors2);
		 
  }
  cout<<"compute pfh features done"<<endl;
  // Find feature correspondences
  std::vector<int> correspondences;
  std::vector<float> correspondence_scores;
  cout<<"find correspondences"<<endl;
  find_feature_correspondences (descriptors1, descriptors2, correspondences, correspondence_scores);
 
		
  // Print out ( number of keypoints / number of points )
  std::cout << "First cloud: Found " << keypoints1->size () << " keypoints "
            << "out of " << downsampled1->size () << " total points." << std::endl;
  std::cout << "Second cloud: Found " << keypoints2->size () << " keypoints "
            << "out of " << downsampled2->size () << " total points." << std::endl;


  // Visualize the two point clouds and their feature correspondences
 //visualize_correspondences (points1, keypoints1, points2, keypoints2, correspondences, correspondence_scores);


  pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ2 (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud (*keypoints1, *keypointsXYZ1);
  pcl::copyPointCloud (*keypoints2, *keypointsXYZ2);

  // Get best match correspondences
  vector<int>src_cor, target_cor;
  float correspondence_threshold = 1000;
  vector<int>unique_cor;
  vector<bool> duplicates;

  for(int i=0; i<correspondences.size(); i++)
  {
	  bool duplicate_default = false;
	  int j=0;
	  for(; j<i; j++)
	  {
		  if(correspondences[j] == correspondences[i])
		  {
			  break;
		  }
	  }
	  if(j==i)
	  {
		  duplicates.push_back(false);
	  }
	  else
	  {
		  duplicates.push_back(true);
	  }

  }

  for(int i=0; i<duplicates.size(); i++)
  {
	  if(!duplicates[i])
	  {
		  unique_cor.push_back(correspondences[i]);
	  }
  }

  


  for(int i=0; i<unique_cor.size(); i++)
  {
	  int index=-1;
	  int score = 0;
	  for (int j=0; j<correspondence_scores.size(); j++)
	  {
		  if(unique_cor[i] == correspondences[j])
		  {
			  if(correspondence_scores[j] > score)
			  {
				  index = j;
				  score = correspondence_scores[j];
			  }
		  }
	  }
	  target_cor.push_back(index);
	  src_cor.push_back(unique_cor[i]);
  
  }


  pcl::CorrespondencesPtr pclCor (new pcl::Correspondences());
  pcl::Correspondences inliers;//(new pcl::Correspondences());
  for(int i=0; i<correspondences.size(); i++)
  {
	  pcl::Correspondence corr (correspondences[i], i, correspondence_scores[i]);
	  pclCor->push_back(corr);
  }

  //RUN RANSAC TO REMOVE OUTLIERS (LETS HOPE IT WORKS)
  pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZ> ransac;
  ransac.setInputCloud(keypointsXYZ2);
  ransac.setTargetCloud(keypointsXYZ1);
  ransac.setInlierThreshold(0.05);
  ransac.setMaxIterations(100000);
  ransac.setInputCorrespondences(pclCor);
  cout<<"Starting RANSAC"<<endl;
  ransac.getCorrespondences(inliers);
  cout<<"RANSAC done"<<endl;


 /*  // Create some new point clouds to hold our transformed data
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_left (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_left (new pcl::PointCloud<pcl::PointWithScale>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_right (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_right (new pcl::PointCloud<pcl::PointWithScale>);

  // Shift the first clouds' points to the left
  //const Eigen::Vector3f translate (0.0, 0.0, 0.3);
  const Eigen::Vector3f translate (1, 0.0, 0.0);
  const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
  pcl::transformPointCloud (*pointstemp1, *points_left, -translate, no_rotation);
  pcl::transformPointCloud (*keypointsXYZ1, *keypoints_left, -translate, no_rotation);

  // Shift the second clouds' points to the right
  pcl::transformPointCloud (*pointstemp2, *points_right, translate, no_rotation);
  pcl::transformPointCloud (*keypointsXYZ2, *keypoints_right, translate, no_rotation);

  // Add the clouds to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points_left, "points_left");
 viz.addPointCloud (points_right, "points_right");

  

  // Draw lines between the best corresponding points
  for (size_t i = 0; i < inliers.size(); ++i)
  {
    

    // Get the pair of points
	    pcl::PointXYZ &p__left = keypoints_left->points[inliers[i].index_match];
		pcl::PointXYZRGB p_left;
		p_left.x = p__left.x;
		p_left.y = p__left.y;
		p_left.z = p__left.z;
	  p_left.r = 255;
	  p_left.g = 255;
	  p_left.b = 255;
	  pcl::PointXYZ &p__right = keypoints_right->points[inliers[i].index_match];
    pcl::PointXYZRGB p_right;
	p_right.x = p__right.x;
		p_right.y = p__right.y;
		p_right.z = p__right.z;
	  p_right.r = 255;
	  p_right.g = 255;
	  p_right.b = 255;


    // Generate a random (bright) color
    double r = (rand() % 100);
    double g = (rand() % 100);
    double b = (rand() % 100);
    double max_channel = std::max (r, std::max (g, b));
    r /= max_channel;
    g /= max_channel;
    b /= max_channel;

    // Generate a unique string for each line
    std::stringstream ss ("line");
    ss << i;

    // Draw the line
   viz.addLine (p_left, p_right, r, g, b, ss.str ());
  }

  // Give control over to the visualizer
  viz.spin ();



  */



  TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformer;
  //Eigen::Matrix4f transMat;
  //transformer.estimateRigidTransformation(*keypointsXYZ2, src_cor, *keypointsXYZ1, target_cor, transMat);
  transformer.estimateRigidTransformation(*keypointsXYZ2, *keypointsXYZ1, inliers, transMat);

  return (0);
}

