#ifndef ENV_PROVIDER_HPP
#define ENV_PROVIDER_HPP

#include <uwds/uwds.h>
#include <uwds/uwds_client_nodelet.h>
#include <uwds/tools/model_loader.h>

using namespace uwds_msgs;
using namespace uwds;

namespace uwds_basic_clients
{
  class EnvProvider : public uwds::UwdsClientNodelet
  {
    public:
      /**@brief
       * The constructor.
       */
      EnvProvider(): uwds::UwdsClientNodelet(uwds::PROVIDER) {}

      /**@brief
       * The default destructor
       */
      ~EnvProvider() = default;

      /** @brief
       * Initialize method. Subclass should call this method
       * in its onInit method.
       */
      virtual void onInit();
  };
}

#endif
