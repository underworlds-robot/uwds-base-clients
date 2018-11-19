#ifndef AR_TAG_PROVIDER_HPP
#define AR_TAG_PROVIDER_HPP

#include <uwds/uwds.h>
#include <uwds/uwds_client_nodelet.h>
#include <uwds/tools/model_loader.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

using namespace uwds_msgs;
using namespace uwds;

namespace uwds_basic_clients
{
  class ArObjectsProvider : public uwds::UwdsClientNodelet
  {
    public:
      /**@brief
       * The constructor.
       */
      ArObjectsProvider(): uwds::UwdsClientNodelet(uwds::PROVIDER) {}

      /**@brief
       * The default destructor
       */
      ~ArObjectsProvider() = default;

      /** @brief
       * Initialize method. Subclass should call this method
       * in its onInit method.
       */
      virtual void onInit();

    protected:
      /**@brief
       * This method is called when perception data are received.
       */
      void callback(const ar_track_alvar_msgs::AlvarMarkersConstPtr& msg);

      std::map<int, Node> marker_node_;

      std::map<int, std::vector<uwds_msgs::Mesh>> marker_meshes_;

      std::map<int, bool> meshes_ever_send_;

      std::string input_frame_;

      /**@brief
       * The output world name.
       */
      std::string output_world_;
      /**@brief
       * Input subscriber for perception data.
       */
      ros::Subscriber input_subscriber_;
  };
}

#endif
