#include "uwds_basic_clients/ar_objects_provider.h"

namespace uwds_basic_clients
{
  void ArObjectsProvider::onInit()
  {
    UwdsClientNodelet::onInit();
    pnh_->param<std::string>("output_world", output_world_, "ar_objects");
    std::string marker_ids;
    std::vector<std::string> marker_ids_list;
    if(!pnh_->getParam("marker_ids", marker_ids))
    {
      NODELET_ERROR("[%s] Error occured : Need to specify the marker ids parameter", ctx_->name().c_str());
      return;
    }
    boost::split(marker_ids_list, marker_ids, boost::is_any_of(" "), boost::token_compress_on);

    std::string marker_names;
    std::vector<std::string> marker_names_list;
    if(!pnh_->getParam("marker_names", marker_names))
    {
      NODELET_ERROR("[%s] Error occured : Need to specify the marker names parameter", ctx_->name().c_str());
      return;
    }
    boost::split(marker_names_list, marker_names, boost::is_any_of(" "), boost::token_compress_on);
    if(marker_names_list.size() != marker_names_list.size())
    {
      NODELET_ERROR("[%s] Error occured : Parameters marker_ids and marker_names need to have the same size", ctx_->name().c_str());
      return;
    }
    std::string ressources_folder;
    if(!pnh_->getParam("ressources_folder", ressources_folder))
    {
      NODELET_ERROR("[%s] Error occured : Need to specify the ressource folder parameter", ctx_->name().c_str());
      return;
    }
    std::vector<double> scale;
    scale.push_back(1.0);
    scale.push_back(1.0);
    scale.push_back(1.0);
    std::vector<uwds_msgs::Node> nodes_imported;
    for(unsigned int i = 0; i< marker_ids_list.size(); i++)
    {
      std::vector<uwds_msgs::Mesh> meshes_imported;
      Node new_node;
      new_node.name = marker_names_list[i];
      new_node.id = NEW_UUID;
      std::vector<double> aabb;
      uwds_msgs::Property mesh_property;
      mesh_property.name = "meshes";
      uwds_msgs::Property aabb_property;
      aabb_property.name = "aabb";
      if(ctx_->worlds()[output_world_].pushMeshesFrom3DFile(ressources_folder+"/blend/"+marker_names_list[i]+".blend", meshes_imported, aabb))
      {
        NODELET_INFO("[%s] Mesh '%s' loaded", ctx_->name().c_str(), (ressources_folder+"/blend/"+marker_names_list[i]+".blend").c_str());
        for (unsigned int j = 0; j < meshes_imported.size(); j++)
        {
          mesh_property.data += meshes_imported[j].id;
          if (j<meshes_imported.size()-1)
            mesh_property.data += ",";
        }
        aabb_property.data = to_string(aabb[0]) + "," + to_string(aabb[1]) + "," + to_string(aabb[2]);
        new_node.properties.push_back(mesh_property);
        new_node.properties.push_back(aabb_property);
        new_node.type = MESH;
      }
      marker_node_.emplace(atoi(marker_ids_list[i].c_str()), new_node);
      marker_meshes_.emplace(atoi(marker_ids_list[i].c_str()), meshes_imported);
    }
    pnh_->param<std::string>("input_frame", input_frame_, "camera");
    input_subscriber_ = nh_->subscribe("ar_pose_marker", 1, &ArObjectsProvider::callback, this);
  }

  void ArObjectsProvider::callback(const ar_track_alvar_msgs::AlvarMarkersConstPtr& msg)
  {
    if(msg->markers.size()>0)
    {
      Changes changes;
      for (const auto& marker : msg->markers)
      {
        if(marker_node_.count(marker.id))
        {
          marker_node_[marker.id].position.pose = marker.pose.pose;
          Node node = marker_node_[marker.id];
          node.last_observation.data = ros::Time::now();
          if(!meshes_ever_send_.count(marker.id))
          {
            for (const auto& meshes_pair : marker_meshes_)
            {
              for (const auto& mesh : meshes_pair.second)
                changes.meshes_to_update.push_back(mesh);
            }
            meshes_ever_send_.emplace(marker.id, true);
          }
          changes.nodes_to_update.push_back(node);
        }
      }
      ctx_->worlds()[output_world_].update(msg->markers[0].header, changes);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds_basic_clients::ArObjectsProvider, nodelet::Nodelet)
