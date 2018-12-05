#include "uwds_basic_clients/world_merger_nodelet.h"

namespace uwds_basic_clients
{
  void WorldMerger::onInit()
  {
    uwds::ReconfigurableClient::onInit();
  }

  void WorldMerger::onChanges(const std::string& world,
                              const std_msgs::Header header,
                              const std::vector<std::string> nodes_id_updated,
                              const std::vector<std::string> nodes_id_deleted,
                              const std::vector<std::string> situations_id_updated,
                              const std::vector<std::string> situations_id_deleted,
                              const std::vector<std::string> meshes_id_updated,
                              const std::vector<std::string> meshes_id_deleted)
  {
    Invalidations invalidations;
    invalidations.node_ids_updated = nodes_id_updated;
    invalidations.node_ids_deleted = nodes_id_deleted;
    invalidations.situation_ids_updated = situations_id_updated;
    invalidations.situation_ids_deleted = situations_id_deleted;
    invalidations.mesh_ids_updated = meshes_id_updated;
    invalidations.mesh_ids_deleted = meshes_id_deleted;
    onChanges(world, header, invalidations);
  }

  void WorldMerger::onChanges(const std::string& world,
                              const std_msgs::Header& header,
                              const Invalidations& invalidations)
  {
    auto& scene = worlds()[world].scene();
    auto& timeline = worlds()[world].timeline();
    auto& meshes = worlds()[world].meshes();

    Changes changes;

    for(const std::string& id : invalidations.node_ids_updated)
    {
      if (scene.nodes()[id].name != "root")
        changes.nodes_to_update.push_back(scene.nodes()[id]);
    }
    for(const std::string& id : invalidations.situation_ids_updated)
    {
      changes.situations_to_update.push_back(timeline.situations()[id]);
    }
    for(const std::string& id : invalidations.mesh_ids_updated)
    {
      changes.meshes_to_update.push_back(meshes[id]);
    }
    sendWorldChanges(output_world_, header, changes);
    if(verbose_)NODELET_INFO("[%s::onChanges] Send changes to world <%s>", nodelet_name_.c_str(), output_world_.c_str());
  }

  void WorldMerger::onReconfigure(const std::vector<std::string>& new_input_worlds) {}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds_basic_clients::WorldMerger, nodelet::Nodelet)
