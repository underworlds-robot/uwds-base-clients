#include "uwds_basic_clients/world_merger_nodelet.h"

namespace uwds_basic_clients
{
  void WorldMerger::onInit()
  {
    uwds::ReconfigurableClient::onInit();
  }

  void WorldMerger::onChanges(const string& world,
                              const Header& header,
                              const Invalidations& invalidations)
  {
    auto& scene = ctx_->worlds()[world].scene();
    auto& timeline = ctx_->worlds()[world].timeline();
    auto& meshes = ctx_->worlds()[world].meshes();

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
    ctx_->worlds()[output_world_].update(header, changes);
    if(verbose_)NODELET_INFO("[%s::onChanges] Send changes to world <%s>", ctx_->name().c_str(), output_world_.c_str());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds_basic_clients::WorldMerger, nodelet::Nodelet)
