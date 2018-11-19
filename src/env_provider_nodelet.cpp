#include "uwds_basic_clients/env_provider_nodelet.h"

namespace uwds_basic_clients
{
  void EnvProvider::onInit()
  {
    uwds::UwdsClientNodelet::onInit();

    std::string filename;
    pnh_->param<std::string>("filename", filename, "");

    std::string output_world;
    pnh_->param<std::string>("output_world", output_world, "env");

    bool only_meshes;
    pnh_->param<bool>("only_meshes", only_meshes, false);

    float scale;
    pnh_->param<float>("scale", scale , 1.0);

    std::vector<Mesh> meshes_imported;
    std::vector<Node> nodes_imported;
    if (!ModelLoader().loadScene(filename, "", only_meshes, meshes_imported, nodes_imported))
    {
      NODELET_ERROR("[%s] Error while loading file !", nodelet_name_.c_str());
      return;
    } else {
      NODELET_INFO("[%s] File '%s' loaded successfully !", nodelet_name_.c_str(), filename.c_str());
    }

    connection_status_ = CONNECTED;
    ros::Duration(0.5).sleep();

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    Changes changes;
    changes.nodes_to_update = nodes_imported;
    changes.meshes_to_update = meshes_imported;
    sendWorldChanges(output_world, header, changes);
    NODELET_INFO("[%s] Environment ready !", nodelet_name_.c_str());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds_basic_clients::EnvProvider, nodelet::Nodelet)
