#ifndef WORLD_MERGER_HPP
#define WORLD_MERGER_HPP

#include <uwds/uwds.h>
#include <uwds/reconfigurable_client.h>

using namespace uwds_msgs;
using namespace uwds;
using namespace std_msgs;
using namespace std;

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
     * @param header The header
     * @param invalidations The invalidations received
     */
    virtual void onChanges(const string& world,
                           const Header& header,
                           const Invalidations& invalidations);

    virtual void onReconfigure(const std::vector<std::string>& worlds) {}
  };
}

#endif
