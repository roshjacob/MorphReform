//#include "stdafx.h"
//#include <iostream>
//#include "random_sample_consensus.h"
//#include <pcl/io/vtk_lib_io.hpp>
//#include <pcl/console/parse.h>
//#include <afxwin.h>
//PCL.VTK MFC integration
/*vtkSmartPointer<vtkRenderer> ren1;
vtkSmartPointer<vtkRenderWindow> renwin;
vtkSmartPointer<vtkRenderWindowInteractor> iren;
pcl::visualization::PCLVisualizer viewer("3D Viewer" , false);
extern CStatic *pclStatic;
extern LPRECT rect;*/
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/registration/distances.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/model_types.h>
#include <boost/thread/thread.hpp>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
typedef pcl::PointXYZ PointT;


extern bool bCheckPolyView, bRunRansac, bExit,bMorphHeadRadius;
int count=0, mode =0;
double neck_angle, neck_length, head_radius, head_radius_old;
  // Objects and Datasets Declaration
 // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
 
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);  // initialize PointClouds
  pcl::PointCloud<PointT>::Ptr cloud_sphere (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  Eigen::Vector4f sphere_centroid, cylinder_centroid, cylinder_centroid_min;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PolygonMesh mesh;
  //boost::shared_ptr<pcl::PolygonMesh> mesh;
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices);
 
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red_color(cloud, 255, 0, 0);//RED COLOR Quad
  pcl::visualization::PointCloudColorHandlerCustom<PointT> green_color(cloud, 0, 255, 0);//GREEN COLOR Quad
  pcl::visualization::PointCloudColorHandlerCustom<PointT> blue_color(cloud, 0, 0, 255);//BLUE COLOR Quad
  pcl::visualization::PointCloudColorHandlerCustom<PointT> yellow_color(cloud, 255, 255, 0);//Yellow COLOR Quad
  // --------------------------------------------
  // -----Open simpleVis and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
  {
	 // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	  viewer->setBackgroundColor (0, 0, 0);
	  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	  //viewer->addCoordinateSystem (1.0);
	  viewer->initCameraParameters ();
	  //viewer->setCameraPosition(0,0,1,0,0,1,0);
	  //770.508,1207.07/86.1482,2.9314,194.458/-480.355,-773.371,252.114/0.0365319,0.0474783,0.998204/0.523599/727,824/0,0
	    viewer->camera_.view[0] = 0.0365319;
		viewer->camera_.view[1] = 0.0474783;
		viewer->camera_.view[2] = 0.998204;
		viewer->camera_.pos[0] = -480.355;
		viewer->camera_.pos[1] = -773.371;
		viewer->camera_.pos[2] = 252.114;
		viewer->camera_.focal[0] = 86.1482;
		viewer->camera_.focal[1] = 2.9314;
		viewer->camera_.focal[2] = 194.458;
		viewer->camera_.window_size[0] = 700;
		viewer->camera_.window_size[1] = 824;
	    viewer->updateCamera();
	  //viewer->resetCamera();
	  //Eigen::Affine3f vp= viewer->getViewerPose();
	  // viewer->resetCameraViewpoint();
	  return (viewer);	
  }
  // --------------------------------------------
  // -----Open 3D rgbVis and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
  {  
	  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	  viewer->setBackgroundColor (0, 0, 0);
	  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
	  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	  //viewer->addCoordinateSystem (1.0);
	  // viewer->camera_.view
	  viewer->initCameraParameters ();
	  return (viewer);
 }

//MAIN PROGRAM ~ Starts...
int pcl_ransac()
{
	if (bExit)						
	{
			//mode =0; // Close All Windows//
			//vtkSmartPointer<vtkRenderWindow> renwin;
			//vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkRenderWindowInteractor::New();
			//iren->GetRenderWindow()->Finalize();
			//iren->TerminateApp();
			HWND hWnd = (HWND) viewer->getRenderWindow()->GetGenericWindowId();
			viewer->close(); 
			DestroyWindow(hWnd);
			return 0;
	}
	else if (bRunRansac) 
	{
		if(cloud_sphere->points.empty ())	mode =1; // Segmentation First time run, compute model, and display //
		else if (bMorphHeadRadius)	        mode =3; // Perform Morphing //
		else								mode =2; // Segmentation was run before; load saved model, and display //
	}
	else if(!bRunRansac)
	{
		if(cloud_sphere->points.empty ())	mode =4; // Clear Segmentation and just display the STL model //
		else						        mode =4; // Clear Segmentation and just display the STL model //<as of now> same as one above...
	}
	else 
	{
		mode  =4;
	}
	switch(mode)
	{
	case 1:
		  {
			
				//Load STL File to PCL Mesh	
				pcl::io::loadPolygonFileSTL("femur.stl", mesh); 
				pcl::fromROSMsg(mesh.cloud, *cloud);
 
				//.......................................................................................................................................
				//RANSAC - Sphere and Cylinder........................................................................................................
				//.......................................................................................................................................
				std::vector<int> inliers;

				// Estimate point normals
				ne.setSearchMethod (tree);
				ne.setInputCloud (cloud);
				ne.setKSearch (50); //ne.setRadiusSearch(0.05); // 0.005 = 50 mm			
				ne.compute (*cloud_normals);
	
				//Sphere segmentation and set all the parameters
				seg.setOptimizeCoefficients (true);
				seg.setModelType (pcl::SACMODEL_SPHERE);
				seg.setMethodType (pcl::SAC_RANSAC);
				seg.setNormalDistanceWeight (0.1);// Surface Normals influence weight, 0.1
				seg.setMaxIterations (10000);
				seg.setDistanceThreshold (5); // 0.05
				seg.setRadiusLimits (0, 27);// 0.1Radius of cylinder model ( <0.005=50mm )
				seg.setInputCloud (cloud);
				seg.setInputNormals (cloud_normals);
				// Obtain the cylinder inliers and coefficients
				seg.segment (*inliers_sphere, *coefficients_sphere);
				// Extract the cylinder inliers from cloud
				extract.setInputCloud (cloud);
				extract.setIndices (inliers_sphere);
				extract.setNegative (false);	
				extract.filter (*cloud_sphere);
				pcl::compute3DCentroid(*cloud_sphere, sphere_centroid);

				Eigen::Vector4f min_pt_sph,max_pt_sph;
				
				//pcl::getMinMax3D(*cloud_sphere, min_pt_sph, max_pt_sph);
				pcl::getMaxDistance(*cloud_sphere,sphere_centroid,max_pt_sph);
				head_radius =  pcl::distances::l2(sphere_centroid,max_pt_sph);
				head_radius_old = head_radius;
				PointT sphere_min, sphere_max;
				sphere_min.x = min_pt_sph[0]; sphere_min.y = min_pt_sph[1]; sphere_min.z = min_pt_sph[2];
				sphere_max.x = max_pt_sph[0]; sphere_max.y = max_pt_sph[1]; sphere_max.z = max_pt_sph[2];
				//viewer->addSphere(sphere_min,03,1,1,0,"sphere_min",0);
				viewer->addSphere(sphere_max,01,1,1,0,"sphere_max",0);
				
				
	
				//Cylinder segmentation and set all the parameters			
				seg.setOptimizeCoefficients (true);
				seg.setModelType (pcl::SACMODEL_CYLINDER);
				seg.setMethodType (pcl::SAC_RANSAC);
				seg.setNormalDistanceWeight (0.1);// Surface Normals influence weight, 0.1
				seg.setMaxIterations (10000);
				seg.setDistanceThreshold (5); // 0.05
				seg.setRadiusLimits (0, 30);// 0.1Radius of cylinder model ( <0.005=50mm )
				seg.setInputCloud (cloud);
				seg.setInputNormals (cloud_normals);
				// Obtain the cylinder inliers and coefficients
				seg.segment (*inliers_cylinder, *coefficients_cylinder);
				// Extract the cylinder inliers from cloud
				extract.setInputCloud (cloud);
				extract.setIndices (inliers_cylinder);
				extract.setNegative (false);																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																										
				extract.filter (*cloud_cylinder);
				
				//pcl::compute3DCentroid(*cloud_cylinder, cylinder_centroid); // All points centroid
				Eigen::Vector4f min_pt_cyl,max_pt_cyl;
				float MIN,MAX;
				PointT xphere, xylinder_min, xylinder_max;
			    pcl::getMinMax3D(*cloud_cylinder,min_pt_cyl,max_pt_cyl);
				// Build a passthrough filter to extract bottom 25% points
				pass.setInputCloud (cloud_cylinder);
				pass.setFilterFieldName ("z");
				MAX = (float)min_pt_cyl.coeff(2)+(0.25*((float)max_pt_cyl.coeff(2)-(float)min_pt_cyl.coeff(2))) ;
				pass.setFilterLimits ((float)min_pt_cyl.coeff(2),(float)MAX);
				pass.filter (*cloud_filtered);
				pcl::compute3DCentroid(*cloud_filtered, cylinder_centroid_min); // Bottom points centroid
				xylinder_min.x = cylinder_centroid_min[0]; xylinder_min.y = cylinder_centroid_min[1]; xylinder_min.z = cylinder_centroid_min[2];//min_pt_cyl.coeff(2);
				viewer->addSphere(xylinder_min,07,1,1,0,"cylinder_centroid_min",0);
				
				// Build a passthrough filter to extract top 25% points
				pass.setInputCloud (cloud_cylinder);
				pass.setFilterFieldName ("z");
				MIN = (float)max_pt_cyl.coeff(2)-(0.25*((float)max_pt_cyl.coeff(2)-(float)min_pt_cyl.coeff(2)));
				pass.setFilterLimits ((float)MIN, (float)max_pt_cyl.coeff(2));
				pass.filter (*cloud_filtered);
				pcl::compute3DCentroid(*cloud_filtered, cylinder_centroid); // Top points centroid
				xylinder_max.x = cylinder_centroid[0]; xylinder_max.y = cylinder_centroid[1]; xylinder_max.z = cylinder_centroid[2];// max_pt_cyl.coeff(2);
				viewer->addSphere(xylinder_max,07,1,0,1,"cylinder_centroid_max",0);
				// creates the visualization object and adds either our orignial cloud or all of the inliers
	
				//DISPLAY on VTk....
				if(bCheckPolyView)  
					viewer->addPolygonMesh(mesh,"polygon",0);
				else			    
					viewer->removePolygonMesh("polygon",0);
				viewer = simpleVis(cloud);
				viewer->addPointCloud<PointT>(cloud_cylinder, green_color, "cylinder");
				viewer->addPointCloud<PointT>(cloud_sphere, red_color, "sphere");
				//PointT xphere, xylinder_min, xylinder_max;
				xphere.x = sphere_centroid[0]; xphere.y = sphere_centroid[1]; xphere.z = sphere_centroid[2];	
				//xylinder_min.x = cylinder_centroid[0]; xylinder_min.y = cylinder_centroid[1]; xylinder_min.z = min_pt_cyl.coeff(2);
				//xylinder_max.x = cylinder_centroid[0]; xylinder_max.y = cylinder_centroid[1]; xylinder_max.z = max_pt_cyl.coeff(2);
				//xylinder_min.x = min_pt_cyl.coeff(0); xylinder_min.y = min_pt_cyl.coeff(1); xylinder_min.z = min_pt_cyl.coeff(2);
				//xylinder_max.x = max_pt_cyl.coeff(0); xylinder_max.y = max_pt_cyl.coeff(1); xylinder_max.z = max_pt_cyl.coeff(2);
				viewer->addSphere(xphere,07,0,1,0,"sphere_centroid",0);
				//xylinder_max.z+=150; 
				//xylinder_min.z-=50;
				viewer->addLine(xylinder_min,xylinder_max,0,0,1,"Cylinder_Axis",0);
				viewer->addLine(xphere, xylinder_max,0,0,1,"Sphere_Axis",0);
				
				//cylinder_centroid_min cylinder_centroid sphere_centroid
				
				Eigen::Vector4f a = sphere_centroid - cylinder_centroid;
				
				Eigen::Vector4f b = cylinder_centroid - cylinder_centroid_min ;
				
				neck_angle = pcl::rad2deg((double)pcl::getAngle3D(a,b));
				neck_length = pcl::distances::l2(sphere_centroid,cylinder_centroid);
				return 0;

				/*pcl::ModelCoefficients line_coeff_sphere;
				line_coeff_sphere.values.resize(6);
				line_coeff_sphere.values[0] = coefficients_sphere->values[0];
				line_coeff_sphere.values[1] = coefficients_sphere->values[1];
				line_coeff_sphere.values[2] = coefficients_sphere->values[2];

				line_coeff_sphere.values[3] = coefficients_sphere->values[3];
				line_coeff_sphere.values[4] = coefficients_sphere->values[4];
				line_coeff_sphere.values[5] = coefficients_sphere->values[5];
				viewer->addLine(line_coeff_sphere);//,"line",0);*/
				break;
			 
		  }//end of case 1;
	case 2:
		  {
				if(bCheckPolyView) 
					viewer->addPolygonMesh(mesh,"polygon",0);
				else			   
					viewer->removePolygonMesh("polygon",0);
				viewer = simpleVis(cloud);
				viewer->addPointCloud<PointT>(cloud_cylinder, green_color, "cylinder");
				viewer->addPointCloud<PointT>(cloud_sphere, red_color, "sphere");
				PointT xphere ;
				xphere.x = sphere_centroid[0]; xphere.y = sphere_centroid[1];xphere.z = sphere_centroid[2];	
				viewer->addSphere(xphere,07,0,1,0,"sphere_centroid",0);

				break;
		  }
	case 3:
		  {
				 //Morph
			    
				Eigen::Vector4f point, distance;
				float magnitude =0;
				
				for (int i = 0; i< cloud_sphere->points.size(); i++)
				{
					
					point[0] =  cloud_sphere->points[i].x; 
					point[1] =  cloud_sphere->points[i].y; 
					point[2] =  cloud_sphere->points[i].z;
					distance = sphere_centroid - point ;
					magnitude = distance.norm();

					cloud_sphere->points[i].x += (distance[0]/magnitude) * ( head_radius_old - head_radius);
					cloud_sphere->points[i].y += (distance[1]/magnitude) * ( head_radius_old - head_radius);
					cloud_sphere->points[i].z += (distance[2]/magnitude) * ( head_radius_old - head_radius);

				}
				head_radius_old = head_radius;

				 //Display PCL Mesh	
				//viewer->removePointCloud("sphere");
				//viewer->removeAllShapes();
				
				if(bCheckPolyView) 
					viewer->addPolygonMesh(mesh,"polygon",0);
				else			   
					viewer->removePolygonMesh("polygon",0);
				//viewer = simpleVis(cloud);
				//viewer->addPointCloud<PointT>(cloud_cylinder, green_color, "cylinder");
				viewer->removePointCloud("sphere_new");
				viewer->addPointCloud<PointT>(cloud_sphere, yellow_color, "sphere_new");
				//PointT xphere ;
				//xphere.x = sphere_centroid[0]; xphere.y = sphere_centroid[1];xphere.z = sphere_centroid[2];	
				//viewer->addSphere(xphere,07,0,1,0,"sphere_centroid",0);
				return 0;

				break;
		  }
	case 4:
		  {
				 //Load STL File to PCL Mesh	
			 	 pcl::io::loadPolygonFileSTL("femur.stl", mesh); 
				 pcl::fromROSMsg(mesh.cloud, *cloud);
				 viewer->removeAllPointClouds();
				 viewer->removeAllShapes();
				 viewer = simpleVis(cloud);
				 if(bCheckPolyView) 
					viewer->addPolygonMesh(mesh,"polygon",0);
				else			   
					viewer->removePolygonMesh("polygon",0);

				 break;
		  }
	default:
		  {
				 viewer->close();
				 // viewer->wasStopped();
				 break;
		  }
    }// end of Switch Statement //



	 //viewer->removePointCloud("sample cloud" ,0);
	// viewer = rgbVis(cloudrgb);
	
    //.........................................................................................................................
    //POLYGON MESH VIEWER
    //.........................................................................................................................
    //pcl::visualization::PCLVisualizer viewer;
	//viewer->addPolygonMesh(mesh,"polygon",0);
	//if (!viz.updatePolygonMesh<pcl::PointXYZ> (cloud, model_polygon_mesh_->polygons, "polygon")) 
		//viz.addPolygonMesh<pcl::PointXYZ> (cloud, model_polygon_mesh_->polygons, "polygon", 0);
    //if(viewer->updatePolygonMesh<pcl::PointXYZ> (cloud, mesh.polygons,"polygon"))
		//viewer->addPolygonMesh<pcl::PointXYZ>(cloud, mesh.polygons, "polygon",0);
		//viewer.addPolygonMesh(mesh,"polygon",0);
	 //Pass Filter..............................................................................................................
	  /*pcl::PassThrough<PointT> pass;
	  pass.setInputCloud (cloud);
	  pass.setFilterFieldName ("z");
	  pass.setFilterLimits (-50, 1000);
	  pass.filter (*final);*/
	
	
	//......................................................................................................
	// Attach viewer to Dialog box..........................................................................
	//......................................................................................................
	/*
	renwin = viewer.getRenderWindow();
	//static_cast<vtkRenderWindow*> (viewer->getRenderWindow);
	renwin->SetParentId(pclStatic->m_hWnd);
	iren = vtkRenderWindowInteractor::New();
	pclStatic->GetWindowRect(rect);
	renwin->SetSize(rect->right-rect->left, rect->bottom-rect->top);
	renwin->SetPosition(0,0);
	iren->SetRenderWindow(renwin);
	viewer.setBackgroundColor (0, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//viewer->addCoordinateSystem (1.0);
	viewer.initCameraParameters ();
	renwin->Render();
    ...........................................................................................................*/
 
	while (!viewer->wasStopped ())
	{
	viewer->spinOnce (100);
	boost::this_thread::sleep (boost::posix_time::microseconds (100));
	}		
  return 0;
 }
