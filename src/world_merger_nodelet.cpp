#include "uwds_basic_clients/world_merger_nodelet.h"

namespace uwds_basic_clients
{
  void WorldMerger::onInit()
  {
    ros::param::param<std::string>("~output_world", output_world_, "merged");
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
  }

  void WorldMerger::onReconfigure(const std::vector<std::string>& new_input_worlds)
  {
    Changes changes;

    for(const std::string& world : new_input_worlds)
    {
      std::vector<std::string> mesh_ids;
      auto& scene = worlds()[world].scene();
      std::queue<Node> fifo;
      fifo.push(scene.nodes()[scene.rootID()]);
      do
      {
        Node current_node = fifo.front();
        fifo.pop();
        if(current_node.name !="root")
          changes.nodes_to_update.push_back(current_node);
        for (const auto& child_id : current_node.children)
        {
          fifo.push(scene.nodes()[child_id]);
        }
      } while (!fifo.empty());

      for(const auto situation : worlds()[world].timeline().situations())
      {
        changes.situations_to_update.push_back(*situation);
      }
      for(const auto mesh_id : mesh_ids)
      {
        changes.meshes_to_update.push_back(worlds()[world].meshes()[mesh_id]);
      }
    }

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    sendWorldChanges(output_world_,
                     header,
                     changes);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds_basic_clients::WorldMerger, nodelet::Nodelet)
