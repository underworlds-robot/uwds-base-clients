#ifndef WORLD_MERGER_HPP
#define WORLD_MERGER_HPP

#include <uwds/uwds.h>
#include <uwds/reconfigurable_client.h>

using namespace uwds_msgs;
using namespace uwds;

namespace uwds_basic_clients
{
  class WorldMerger : public uwds::ReconfigurableClient
  {
  public:
    /**@brief
     * The default constructor.
     */
    WorldMerger(): uwds::ReconfigurableClient(uwds::FILTER) {}

    /**@brief
     * The default destructor
     */
    ~WorldMerger() = default;

    /** @brief
     * Initialize method. Subclass should call this method
     * in its onInit method.
     */
    virtual void onInit();

  protected:

    /** @brief
     * This method is called when there is a change in a world.
     *
     * @param world The world that have been updated
     * @param nodes_id_updated The node IDs that have been updated
     * @param situations_id_updated The situation IDs that have been updated
     */
    virtual void onChanges(const std::string& world,
                           const std_msgs::Header header,
                           const std::vector<std::string> nodes_id_updated,
                           const std::vector<std::string> nodes_id_deleted,
                           const std::vector<std::string> situations_id_updated,
                           const std::vector<std::string> situations_id_deleted,
                           const std::vector<std::string> meshes_id_updated,
                           const std::vector<std::string> meshes_id_deleted);

    /** @brief
     * This method is called when there is a change in a world.
     *
     * @param world The world that have been updated
     * @param header The header
     * @param invalidations The invalidations received
     */
    virtual void onChanges(const std::string& world,
                           const std_msgs::Header& header,
                           const Invalidations& invalidations);

    virtual void onSubscribeChanges(const std::string world) {}

    virtual void onUnsubscribeChanges(const std::string world) {}

    virtual void onReconfigure(const std::vector<std::string>& worlds);

    std::string output_world_;
  };
}

#endif
