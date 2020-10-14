//=================================================================================================
// Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef VIGIR_CLOUD_TO_MESH_ROS_H_
#define VIGIR_CLOUD_TO_MESH_ROS_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <shape_msgs/Mesh.h>

#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vigir_point_cloud_proc/cloud_to_mesh.h>
#include <vigir_point_cloud_proc/mesh_conversions.h>
//#include <vigir_filtered_localized_scan_utils/filtered_localized_scan_converter.h>

#include "map_msgs/SaveMap.h"

namespace vigir_point_cloud_proc
{

/**
 * @brief The CloudToMeshRos class provides
 * a ROS(topic) interface for converting point clouds
 * messages to mesh representations.
 */
template <typename PointT>
class CloudToMeshRos
{
public:
  CloudToMeshRos()
  {
    ros::NodeHandle pnh("~");

    pnh.param("cloud_sub_queue_size", p_cloud_queue_size_, 1);
    pnh.param("radius_MLS", radius_MLS_, 0.5);
    pnh.param("polynomial_order_MLS", polynomial_order_MLS_, 2);
    pnh.param("voxel_size", voxel_size_, 0.1);
    pnh.param("radius_GPT", radius_GPT_, 0.2);
    pnh.param("maximum_nearest_neighbors_GPT", maximum_nearest_neighbors_GPT_, 200);

    ROS_INFO("CloudToMeshRos using queue size %d", p_cloud_queue_size_);

    marker_pub_ = pnh.advertise<visualization_msgs::Marker>("mesh_marker", 1, true);
    shape_pub_  = pnh.advertise<shape_msgs::Mesh>("mesh_shape", 1, true);

    cloud_sub_ = pnh.subscribe("cloud", p_cloud_queue_size_, &CloudToMeshRos::cloudCallback, this);

    obj_map_server_ = pnh.advertiseService("save_OBJ_map", &CloudToMeshRos::saveOBJMap, this);
    gltf_map_server_ = pnh.advertiseService("save_GLTF_map", &CloudToMeshRos::saveGLTFMap, this);

    cloud_to_mesh_.setVoxelFilterSize(voxel_size_);

    cloud_to_mesh_.setRadiusMLS(radius_MLS_);
    cloud_to_mesh_.setPolynomialOrderMLS(polynomial_order_MLS_);
    cloud_to_mesh_.setRadiusGPT(radius_GPT_);
    cloud_to_mesh_.setMaximumNearestNeighborsGPT(maximum_nearest_neighbors_GPT_);

  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
  {
    boost::shared_ptr<pcl::PointCloud<PointT> > pc (new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_in, *pc);

    cloud_to_mesh_.setInput(pc);

    if (cloud_to_mesh_.computeMesh())
    {

      if (marker_pub_.getNumSubscribers() > 0){
        visualization_msgs::Marker mesh_marker;

        meshToMarkerMsg(cloud_to_mesh_.getMesh() ,mesh_marker);
        marker_pub_.publish(mesh_marker);
      }

      if (shape_pub_.getNumSubscribers() > 0){
        shape_msgs::Mesh shape_mesh;

        meshToShapeMsg(cloud_to_mesh_.getMesh() ,shape_mesh);
        shape_pub_.publish(shape_mesh);
      }
    }else{
      ROS_WARN("Could not generate mesh for point cloud!");
    }
  }

  bool saveOBJMap(map_msgs::SaveMap::Request &req,
                  map_msgs::SaveMap::Response &res)
  {
    pcl::PolygonMesh mesh = cloud_to_mesh_.getMesh();

    pcl::io::saveOBJFile(req.filename.data, mesh);

    return true;
  }

  bool saveGLTFMap(map_msgs::SaveMap::Request &req,
                  map_msgs::SaveMap::Response &res)
  {
    pcl::PolygonMesh mesh = cloud_to_mesh_.getMesh();

    std::string GLTFfilename = req.filename.data;
    std::string OBJfilename;
    int len = GLTFfilename.length();
    
    if(GLTFfilename.substr(len-4,len-1)=="gltf"){
      OBJfilename = GLTFfilename.substr(0,len-4)+"obj";
      pcl::io::saveOBJFile(OBJfilename, mesh);

      std::string command_str = "obj2gltf -i "+OBJfilename+" -o "+GLTFfilename;
      ROS_INFO(command_str.c_str());

      system( command_str.c_str() );
    }
    else{
      ROS_WARN("Incompatible file format");
    }

    return true;
  }


private:

  ros::Subscriber cloud_sub_;
  ros::Publisher marker_pub_;
  ros::Publisher shape_pub_;

  ros::ServiceServer obj_map_server_;
  ros::ServiceServer gltf_map_server_;

  sensor_msgs::PointCloud2 cloud_out_;
  sensor_msgs::PointCloud2 cloud_self_filtered_out;

  int p_cloud_queue_size_;

  double radius_MLS_;
  int polynomial_order_MLS_;
  double voxel_size_;
  double radius_GPT_;
  int maximum_nearest_neighbors_GPT_;

  CloudToMesh<PointT, pcl::PointNormal> cloud_to_mesh_;

};

}
#endif
