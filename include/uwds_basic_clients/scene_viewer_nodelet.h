#ifndef SCENE_VIEWER_HPP
#define SCENE_VIEWER_HPP

#include <uwds/uwds.h>
#include <uwds/reconfigurable_client.h>
#include <uwds/tools/model_loader.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/CameraInfo.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace uwds_msgs;
using namespace uwds;

namespace uwds_basic_clients
{
  #define DEFAULT_GLOBAL_FRAME_ID "odom"

  class SceneViewer : public uwds::ReconfigurableClient
  {
    public:
      /**@brief
       * The constructor.
       */
      SceneViewer(): uwds::ReconfigurableClient(uwds::READER) {}

      /**@brief
       * The default destructor
       */
      ~SceneViewer() = default;

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

      /** @brief
       * This method is called when a world changes is subscribed by this nodelet.
       * Set up additional subscribers in this method.
       *
       * @param world The world subscribed
       */
      virtual void onSubscribeChanges(const std::string world);

      /** @brief
       * This method is called when a world changes is unsubscribed by this nodelet.
       * Shut down additional subscribers in this method.
       *
       * @param world The world unsubscribed
       */
      virtual void onUnsubscribeChanges(const std::string world);

      /** @brief
       * This method is called at the end of the reconfigure process.
       */
      virtual void onReconfigure(const std::vector<std::string>& worlds);

      /**@brief
      * Timer to publish the tf frames.
      */
      virtual void onTimer(const ros::TimerEvent& event);

      /** @brief
       * Publish visualisation topics.
       */
      virtual void publishVisualization(const std::string world, const ros::Time stamp);

      virtual std::vector<visualization_msgs::Marker> nodeToMarkers(const std::string world, const Node node, const ros::Time stamp);

      virtual jsk_recognition_msgs::BoundingBox nodeToBoundingBox(const std::string world, const Node node, const ros::Time stamp);

      virtual sensor_msgs::CameraInfo nodeToCameraInfo(const std::string world, const Node node, const ros::Time stamp);

      tf::TransformListener tf_listener_;

    private:
      /** @brief
       * Publish on changes if true.
       */
      bool publish_on_changes_;

      /** @brief
       * The global frame id.
       */
      std::string global_frame_id_;

      /** @brief
       * The last marker ID used.
       */
      unsigned int last_marker_id_ = 0;

      /** @brief
       * The marker ID map by mesh ID.
       */
      std::map<std::string, unsigned int> marker_id_map_;

      /** @brief
       * The marker map by node world.
       */
      std::map<std::string, boost::shared_ptr<ros::Publisher>> markers_publisher_map_;

      /** @brief
       * The bboxes map by world.
       */
      std::map<std::string, boost::shared_ptr<ros::Publisher>> bboxes_publisher_map_;

      /** @brief
       * The cameras map by world+node.id.
       */
      std::map<std::string, boost::shared_ptr<ros::Publisher>> camera_publishers_map_;

      /** @brief
       * The tf broadcaster.
       */
      tf::TransformBroadcaster tf_broadcaster_;

      /** @brief
       * The visualization timer.
       */
      ros::Timer publisher_timer_;
  };
}

#endif
