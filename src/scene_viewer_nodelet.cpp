#include "uwds_basic_clients/scene_viewer_nodelet.h"

namespace uwds_basic_clients
{
  void SceneViewer::onInit()
  {
    uwds::ReconfigurableClient::onInit();

    pnh_->param<std::string>("global_frame_id", global_frame_id_, "odom");
    if (global_frame_id_ == DEFAULT_GLOBAL_FRAME_ID)
      NODELET_WARN("[%s] Using 'odom' as default '~global_frame_id' parameter", nodelet_name_.c_str());

    pnh_->param<bool>("publish_on_changes", publish_on_changes_, false);
    if (!publish_on_changes_)
    {
      float publisher_frequency;
      pnh_->param<float>("publisher_frequency", publisher_frequency, 10.0);
      publisher_timer_ = nh_->createTimer(ros::Duration(1.0/publisher_frequency), &SceneViewer::onTimer, this);
    }
  }

  void SceneViewer::onChanges(const std::string& world,
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

  void SceneViewer::onChanges(const std::string& world,
                         const std_msgs::Header& header,
                         const Invalidations& invalidations)
  {
    if(publish_on_changes_)
      publishVisualization(world, header.stamp);
  }

  void SceneViewer::onSubscribeChanges(const std::string world)
  {
    markers_publisher_map_.emplace(world, boost::make_shared<ros::Publisher>(nh_->advertise<visualization_msgs::MarkerArray>(world+"/meshes", publisher_buffer_size_)));
    bboxes_publisher_map_.emplace(world, boost::make_shared<ros::Publisher>(nh_->advertise<jsk_recognition_msgs::BoundingBoxArray>(world+"/bboxes", publisher_buffer_size_)));
  }

  void SceneViewer::onUnsubscribeChanges(const std::string world)
  {
    marker_id_map_.erase(world);
    bboxes_publisher_map_.erase(world);
  }

  void SceneViewer::onReconfigure(const std::vector<std::string>& worlds) {}

  std::vector<visualization_msgs::Marker> SceneViewer::nodeToMarkers(const std::string world, const Node node, const ros::Time stamp)
  {
    std::vector<visualization_msgs::Marker> markers;
    std::vector<std::string> mesh_ids;

    //mesh_ids = getMeshes(world, node);
    for(auto property : node.properties)
    {
      if(property.name=="meshes")
      {
        boost::split(mesh_ids, property.data, boost::is_any_of(","), boost::token_compress_on);
        break;
      }
    }

    auto& scene = worlds()[world].scene();

    for(auto mesh_id : mesh_ids)
    {
      visualization_msgs::Marker marker;
      if (node.parent == scene.rootID())
        marker.header.frame_id = global_frame_id_;
      else
        marker.header.frame_id = world + "/" + scene.nodes()[node.parent].name;
      marker.header.stamp = stamp;
      if (marker_id_map_.count(world+mesh_id)==0)
        marker_id_map_.emplace(world+mesh_id, last_marker_id_++);
      marker.id = marker_id_map_.at(world+mesh_id);
      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
      marker.pose = node.position.pose;
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      if(worlds()[world].meshes().has(mesh_id))
      {
        const auto& mesh = worlds()[world].meshes()[mesh_id];
        for (const auto& triangle : mesh.triangles)
        {
          geometry_msgs::Point p0, p1, p2;
          std_msgs::ColorRGBA c0, c1, c2;
          p0.x = mesh.vertices[triangle.vertex_indices[0]].x;
          p0.y = mesh.vertices[triangle.vertex_indices[0]].y;
          p0.z = mesh.vertices[triangle.vertex_indices[0]].z;

          c0.r = mesh.vertex_colors[triangle.vertex_indices[0]].r;
          c0.g = mesh.vertex_colors[triangle.vertex_indices[0]].g;
          c0.b = mesh.vertex_colors[triangle.vertex_indices[0]].b;
          c0.a = mesh.vertex_colors[triangle.vertex_indices[0]].a;

          p1.x = mesh.vertices[triangle.vertex_indices[1]].x;
          p1.y = mesh.vertices[triangle.vertex_indices[1]].y;
          p1.z = mesh.vertices[triangle.vertex_indices[1]].z;

          c1.r = mesh.vertex_colors[triangle.vertex_indices[1]].r;
          c1.g = mesh.vertex_colors[triangle.vertex_indices[1]].g;
          c1.b = mesh.vertex_colors[triangle.vertex_indices[1]].b;
          c1.a = mesh.vertex_colors[triangle.vertex_indices[1]].a;

          p2.x = mesh.vertices[triangle.vertex_indices[2]].x;
          p2.y = mesh.vertices[triangle.vertex_indices[2]].y;
          p2.z = mesh.vertices[triangle.vertex_indices[2]].z;

          c2.r = mesh.vertex_colors[triangle.vertex_indices[2]].r;
          c2.g = mesh.vertex_colors[triangle.vertex_indices[2]].g;
          c2.b = mesh.vertex_colors[triangle.vertex_indices[2]].b;
          c2.a = mesh.vertex_colors[triangle.vertex_indices[2]].a;

          marker.points.push_back(p0);
          marker.colors.push_back(c0);

          marker.points.push_back(p1);
          marker.colors.push_back(c1);

          marker.points.push_back(p2);
          marker.colors.push_back(c2);
        }
        markers.push_back(marker);
      }
    }
    return markers;
  }

  jsk_recognition_msgs::BoundingBox SceneViewer::nodeToBoundingBox(const std::string world, const Node node, const ros::Time stamp)
  {
    jsk_recognition_msgs::BoundingBox bbox;
    std_msgs::Header bbox_header;
    bbox.header.stamp = stamp;
    bbox.header.frame_id = global_frame_id_;
    try
    {
      tf::StampedTransform transform;
      tf_listener_.lookupTransform(global_frame_id_, world+"/"+node.name,
                               ros::Time(), transform);

      tf::Vector3 t = transform.getOrigin();
      tf::Quaternion q = transform.getRotation();
      bbox.pose.position.x = t.getX();
      bbox.pose.position.y = t.getY();
      bbox.pose.position.z = t.getZ();

      bbox.pose.orientation.x = q.getX();
      bbox.pose.orientation.y = q.getY();
      bbox.pose.orientation.z = q.getZ();
      bbox.pose.orientation.w = q.getW();
    } catch(std::exception){
      bbox.header.frame_id = world+"/"+node.name;
    }
    for(const auto& property : node.properties)
    {
      if (property.name == "aabb")
      {
        std::vector<std::string> aabb_data;
        boost::split(aabb_data, property.data, boost::is_any_of(","), boost::token_compress_on);
        if (aabb_data.size() == 3)
        {
          bbox.dimensions.x = std::atof(aabb_data[0].c_str());
          bbox.dimensions.y = std::atof(aabb_data[1].c_str());
          bbox.dimensions.z = std::atof(aabb_data[2].c_str());
        }
      }
    }
    return bbox;
  }

  sensor_msgs::CameraInfo SceneViewer::nodeToCameraInfo(const std::string world, const Node node, const ros::Time stamp)
  {
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = world + "/" + node.name;
    camera_info.header.stamp = stamp;
    camera_info.height=480;
    camera_info.width=640;
    camera_info.distortion_model="plumb_bob";
    camera_info.D.push_back(0.007313);
    camera_info.D.push_back(-0.068372);
    camera_info.D.push_back(-0.002248);
    camera_info.D.push_back(0.001679);
    camera_info.D.push_back(0.0);
    camera_info.K[0] = 509.015381;
    camera_info.K[0] = 0.0;
    camera_info.K[0] = 320.675346;
    camera_info.K[0] = 0.0;
    camera_info.K[0] = 508.683267;
    camera_info.K[0] = 236.130072;
    camera_info.K[0] = 0.0;
    camera_info.K[0] = 0.0;
    camera_info.K[0] = 1.0;

    camera_info.R[0] = 0.999627;
    camera_info.R[1] = 0.007759;
    camera_info.R[2] = 0.026199;
    camera_info.R[3] = -0.007718;
    camera_info.R[4] = 0.999969;
    camera_info.R[5] = -0.001661;
    camera_info.R[6] = -0.026211;
    camera_info.R[7] = 0.001459;
    camera_info.R[8] = 0.999655;

    camera_info.P[0] = 514.03069;
    camera_info.P[1] = 0.0;
    camera_info.P[2] = 302.063255;
    camera_info.P[3] = 0.0;
    camera_info.P[4] = 0.0;
    camera_info.P[5] = 514.03069;
    camera_info.P[6] = 236.707602;
    camera_info.P[7] = 0.0;
    camera_info.P[8] = 0.0;
    camera_info.P[9] = 0.0;
    camera_info.P[10] = 1.0;
    camera_info.P[11] = 0.0;
    return camera_info;
  }

  void SceneViewer::publishVisualization(const std::string world, const ros::Time stamp)
  {
    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = global_frame_id_;

    jsk_recognition_msgs::BoundingBoxArray bboxes;
    visualization_msgs::MarkerArray markers;
    bboxes.header = header;
    auto& scene = worlds()[world].scene();

    for (const auto& node_ptr : scene.nodes())
    {
      Node node = *node_ptr;
      if (node.name != "root")
      {
        std::string source_frame;
        if (node.parent != scene.rootID())
        {
          source_frame = world + "/" + scene.nodes()[node.parent].name;
        } else {
          source_frame = global_frame_id_;
        }
        tf::Transform transform;

        tf::Vector3 t(node.position.pose.position.x,
                      node.position.pose.position.y,
                      node.position.pose.position.z);
        tf::Quaternion q(node.position.pose.orientation.x,
                         node.position.pose.orientation.y,
                         node.position.pose.orientation.z,
                         node.position.pose.orientation.w);
        q.normalize();
        transform.setOrigin(t);
        transform.setRotation(q);

        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, stamp, source_frame, world + "/" + node.name));

        if (node.type == MESH)
        {
          for(auto marker : nodeToMarkers(world, node, stamp))
          {
            markers.markers.push_back(marker);
          }
          bboxes.boxes.push_back(nodeToBoundingBox(world, node, stamp));
        }

        if (node.type == CAMERA)
        {
          if(camera_publishers_map_.count(world+node.id) == 0)
            camera_publishers_map_.emplace(world+node.id, boost::make_shared<ros::Publisher>(nh_->advertise<sensor_msgs::CameraInfo>(world+"/"+node.name+"/camera_info", publisher_buffer_size_)));

          camera_publishers_map_.at(world+node.id)->publish(nodeToCameraInfo(world, node, stamp));
        }
      }
    }
    if (markers_publisher_map_.count(world) > 0  && markers.markers.size()>0)
      markers_publisher_map_.at(world)->publish(markers);

    if (bboxes_publisher_map_.count(world) > 0 && bboxes.boxes.size()>0)
      bboxes_publisher_map_.at(world)->publish(bboxes);
  }

  void SceneViewer::onTimer(const ros::TimerEvent& event) {
    for (const auto& world : input_worlds_)
    {
      ros::Time stamp = ros::Time::now();
      publishVisualization(world, stamp);
    }
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds_basic_clients::SceneViewer, nodelet::Nodelet)
