#include "uwds_basic_clients/simple_object_provider.h"
#include <stdlib.h>

namespace uwds_basic_clients
{
  void SimpleObjectProvider::onInit()
  {
    UwdsClientNodelet::onInit();

    Property meshes_property;
    meshes_property.name = "meshes";
    Property aabb_property;
    aabb_property.name = "aabb";

    pnh_->param<std::string>("object_name", object_name_, "unknown");

    std::string object_model;
    pnh_->param<std::string>("object_model", object_model, "");
    if (object_model != "")
    {
      use_mesh_ = true;
      ModelLoader ml;
      std::vector<double> scale, aabb;
      scale.push_back(1.0);
      scale.push_back(1.0);
      scale.push_back(1.0);

      if(ml.loadMeshes(object_model, scale, object_meshes_, aabb))
      {
        for (unsigned int i=0 ; i < object_meshes_.size() ; i++)
        {
          meshes_property.data += object_meshes_[i].id;
          if (i < object_meshes_.size()-1)
          {
            meshes_property.data += ",";
          }
        }
        aabb_property.data = to_string(aabb[0]) + "," + to_string(aabb[1]) + "," + to_string(aabb[2]);
      } else {
        NODELET_ERROR("[%s] Error while loading file '%s'", ctx_->name().c_str(), object_model.c_str());
        return;
      }
    }
    object_node_.id = NEW_UUID;
    object_node_.name = object_name_;
    object_node_.type = MESH;
    object_node_.properties.push_back(meshes_property);
    object_node_.properties.push_back(aabb_property);
    pnh_->param<std::string>("output_world", output_world_, "simple_object");
    pnh_->param<std::string>("global_frame_id", global_frame_id_, "map");
    input_subscriber_ = pnh_->subscribe("input", 1, &SimpleObjectProvider::callback, this);
  }

  void SimpleObjectProvider::callback(const jsk_recognition_msgs::BoundingBoxConstPtr& msg)
  {
    if (msg->pose.position.x!=0 && msg->pose.position.y!=0 && msg->pose.position.z !=0)
    {
      Changes changes;

      static tf::TransformBroadcaster br;
      tf::Transform box_tr;
      box_tr.setOrigin(tf::Vector3(msg->pose.position.x,
                                      msg->pose.position.y,
                                      msg->pose.position.z));
      box_tr.setRotation(tf::Quaternion(msg->pose.orientation.x,
                                           msg->pose.orientation.y,
                                           msg->pose.orientation.z,
                                           msg->pose.orientation.w));
      br.sendTransform(tf::StampedTransform(box_tr, msg->header.stamp, msg->header.frame_id, object_name_));

      try
      {
        tf::StampedTransform transform;
        tf_listener_.lookupTransform(global_frame_id_, object_name_,
                                 ros::Time(), transform);

        tf::Vector3 t = transform.getOrigin();
        tf::Quaternion q = transform.getRotation();

        ros::Time now = ros::Time::now();
        double deltaX = t.getX() - object_node_.position.pose.position.x;
        double deltaY = t.getY() - object_node_.position.pose.position.y;
        double deltaZ = t.getZ() - object_node_.position.pose.position.z;
        double deltaT;

        if (object_node_.last_observation.data.toSec()!= 0.0)
        {
          if(now.toSec() <= object_node_.last_observation.data.toSec())
            return;
          //deltaT = msg->header.stamp.toSec() - object_node_.last_observation.data.toSec();
          deltaT = now.toSec() - object_node_.last_observation.data.toSec();
        }
        else {
          deltaT = 0.0;
        }

        if(deltaT != 0.0)
        {
          if(deltaX != 0.0)
          {
            object_node_.velocity.twist.linear.x = deltaX / deltaT;
          } else {
            object_node_.velocity.twist.linear.x = 0.0;
          }
          if(deltaY != 0.0)
          {
            object_node_.velocity.twist.linear.y = deltaY / deltaT;
          } else {
            object_node_.velocity.twist.linear.y = 0.0;
          }
          if(deltaZ != 0.0)
          {
            object_node_.velocity.twist.linear.z = deltaZ / deltaT;
          } else {
            object_node_.velocity.twist.linear.z = 0.0;
          }
        } else {
          object_node_.velocity.twist.linear.x = NAN;
          object_node_.velocity.twist.linear.y = NAN;
          object_node_.velocity.twist.linear.z = NAN;
        }

        object_node_.position.pose.position.x = t.getX();
        object_node_.position.pose.position.y = t.getY();
        object_node_.position.pose.position.z = t.getZ();

        object_node_.position.pose.orientation.x = 0.0;
        object_node_.position.pose.orientation.y = 0.0;
        object_node_.position.pose.orientation.z = 0.0;
        object_node_.position.pose.orientation.w = 1.0;

        object_node_.last_observation.data = now;

        if (!use_mesh_)
        {
          for (auto& property : object_node_.properties)
          {
            if (property.name == "aabb")
              property.data = to_string(msg->dimensions.x) + "," + to_string(msg->dimensions.y) + "," + to_string(msg->dimensions.z);
          }
        }
        changes.nodes_to_update.push_back(object_node_);
        if (meshes_ever_send_ == false && use_mesh_==true)
        {
          changes.meshes_to_update = object_meshes_;
          meshes_ever_send_ = true;
        }
        ctx_->worlds()[output_world_].update(msg->header, changes);
      }
      catch (tf::TransformException &ex) {
        NODELET_WARN("[%s] Exception occured : %s",ctx_->name().c_str(), ex.what());
      }
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uwds_basic_clients::SimpleObjectProvider, nodelet::Nodelet)
