#include <algorithm>
#include <boost/iostreams/detail/select.hpp>
#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <thread>
#include "ros/duration.h"

#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

#include <tf2/buffer_core.h>

#include <ros/init.h>
#include <ros/time.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <geometry_msgs/TransformStamped.h>

#include <shape_msgs/Mesh.h>
#include <geometric_shapes/shape_operations.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/ColorRGBA.h>

#include <cnr_tf_named_object_loader/cnr_tf_named_object_loader.h>

using namespace std::chrono;
using namespace std::chrono_literals;

namespace cnr_tf_named_object_loader
{
std::string to_string(const objects_t& vv)
{
  std::string ret = "[";
  for (const auto& v : vv)
  {
    // ret += "{id:" + v.id() + ", reference_frame:" + v.reference_frame() + "}, ";
    ret += "{id:" + v.id + ", reference_frame:" + v.header.frame_id + "}, ";
  }
  return ret + "]";
}

std::string to_string(const tf_named_objects_t& vv)
{
  std::string ret = "[";
  for (const auto& v : vv)
  {
    // ret += "{id:" + v.id() + ", " + " frame:" + v.tf_name() + ", reference_frame:" + v.reference_frame() + "}, ";
    ret += "{id:" + v.id + ", " + " frame:" + v.tf_name() + ", reference_frame:" + v.header.frame_id + "}, ";
  }
  return ret + "]";
}

std::string to_string(const std::vector<std::string>& vv)
{
  std::string ret = "[";
  for (const auto& v : vv)
  {
    ret += v + ",";
  }
  return ret + "]";
}

std::vector<std::string> get_id(const objects_t& vv)
{
  std::vector<std::string> ret;
  for (const auto& v : vv)
  {
    // ret.push_back(v.id());
    ret.push_back(v.id);
  }
  return ret;
}

std::vector<std::string> get_id(const tf_named_objects_t& vv)
{
  std::vector<std::string> ret;
  for (const auto& v : vv)
  {
    // ret.push_back(v.id());
    ret.push_back(v.id);
  }
  return ret;
}

std::vector<std::string> get_frame_id(const tf_named_objects_t& vv)
{
  std::vector<std::string> ret;
  for (const auto& v : vv)
  {
    ret.push_back(v.tf_name());
  }
  return ret;
}
std::vector<std::string> get_reference_frame_id(const tf_named_objects_t& vv)
{
  std::vector<std::string> ret;
  for (const auto& v : vv)
  {
    // ret.push_back(v.reference_frame());
    ret.push_back(v.header.frame_id);
  }
  return ret;
}

std::vector<std::string> get_reference_frame_id(const objects_t& vv)
{
  std::vector<std::string> ret;
  for (const auto& v : vv)
  {
    // ret.push_back(v.reference_frame());
    ret.push_back(v.header.frame_id);
  }
  return ret;
}

// const std::string& object_t::id() const
// {
//   return std::get<0>(*this);
// }

// std::string& object_t::id()
// {
//   return std::get<0>(*this);
// }
// const std::string& object_t::path_to_mesh() const
// {
//   return std::get<1>(*this);
// }
// std::string& object_t::path_to_mesh()
// {
//   return std::get<1>(*this);
// }
// const std::string& object_t::reference_frame() const
// {
//   return std::get<2>(*this);
// }
// std::string& object_t::reference_frame()
// {
//   return std::get<2>(*this);
// }
// const geometry_msgs::Pose& object_t::mesh_pose() const
// {
//   return std::get<3>(*this);
// }
// geometry_msgs::Pose& object_t::mesh_pose()
// {
//   return std::get<3>(*this);
// }

// const Eigen::Vector3d& object_t::scale() const
// {
//   return scale_;
// }

// Eigen::Vector3d& object_t::scale()
// {
//   return scale_;
// }

tf_named_object_t::tf_named_object_t(const object_t& obj)
{
  *this = obj;
}

const std::string& tf_named_object_t::tf_name() const
{
  return tf_name_;
}

std::string& tf_named_object_t::tf_name()
{
  return tf_name_;
}

tf_named_object_t& tf_named_object_t::operator=(const object_t& obj)
{
  // this->id() = obj.id();
  // this->path_to_mesh() = obj.path_to_mesh();
  // this->mesh_pose() = obj.mesh_pose();
  // this->reference_frame() = obj.reference_frame();
  // this->scale() = obj.scale();
  // this->tf_name() = obj.id();
  *dynamic_cast<object_t*>(this) = obj;
  this->tf_name() = obj.id;
  return *this;
}

object_t toCollisionObject(const moveit_msgs::CollisionObject& obj)
{
  object_t ret;
  ret = obj;
  // ret.id() = obj.id;
  // ret.reference_frame() = obj.header.frame_id;

  YAML::Node config = YAML::Load(obj.type.db);
  if (config["path_to_mesh"])
  {
    // ret.path_to_mesh() = config["path_to_mesh"].as<std::string>();
    std::string path_to_mesh = config["path_to_mesh"].as<std::string>();
    // ret.mesh_pose() = obj.mesh_poses.front();

    Eigen::Vector3d scale = Eigen::Vector3d::Ones();
    if (config["scale"])
    {
      if (config["scale"]["x"] && config["scale"]["y"] && config["scale"]["z"])
      {
        if (config["scale"]["x"].IsScalar() && config["scale"]["y"].IsScalar() && config["scale"]["z"].IsScalar())
        {
          // ret.scale() << config["scale"]["x"].as<double>(), config["scale"]["y"].as<double>(),
          // config["scale"]["z"].as<double>();
          scale << config["scale"]["x"].as<double>(), config["scale"]["y"].as<double>(),
              config["scale"]["z"].as<double>();
        }
      }
      size_t i_mesh = 0;
      if (config["mesh_index"])
      {
        i_mesh = config["mesh_index"].as<int>();
      }

      // else
      // {
      //   ret.scale() = Eigen::Vector3d::Ones();
      // }

      shapes::Mesh* m = shapes::createMeshFromResource(path_to_mesh, scale);
      shapes::ShapeMsg mesh_msg;
      shapes::constructMsgFromShape(m, mesh_msg);
      delete m;

      shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
      if ((obj.mesh_poses.size() <= i_mesh) || (obj.mesh_poses.size() - 1 != obj.meshes.size()))
      {
        // TO DO
        if (i_mesh == 0)
        {
          ret.mesh_poses.clear();
          ret.mesh_poses.push_back(geometry_msgs::Pose());
          ret.mesh_poses.back().orientation.w = 1.0;
          ret.meshes.clear();
          ret.meshes.push_back(mesh);
        }
        else
        {
          throw std::runtime_error("The case is not yet supported!");
        }
      }
      else
      {
        ret.mesh_poses.clear();
        ret.meshes.clear();
        for (size_t i = 0; i < obj.mesh_poses.size(); i++)
        {
          ret.mesh_poses.push_back(obj.mesh_poses.at(i));
          ret.meshes.push_back((i != i_mesh) ? obj.meshes.at(i) : mesh);
        }
      }
      // ret.mesh_poses.push_back(pose);
      ret.operation = moveit_msgs::CollisionObject::ADD;
    }
  }
  return ret;
}

// ======================================================================
// ==
// ==
// == TFNamedObjectsManager
// ==
// ==
// ======================================================================

TFNamedObjectsManager::TFNamedObjectsManager()  // : tfBuffer_(ros::Duration(2.0))
{
  tfListener_.reset(new tf2_ros::TransformListener(tfBuffer_));
  ros::Duration(tfBuffer_.getCacheLength().toSec()/100.0).sleep();
};

// moveit_msgs::CollisionObject toCollisionObject(const std::string& collisionObjID, const std::string& path_to_mesh,
//                                                const std::string& reference_frame, const geometry_msgs::Pose& pose,
//                                                const Eigen::Vector3d scale)
// {
//   moveit_msgs::CollisionObject collision_object;

//   collision_object.id = collisionObjID;

//   shapes::Mesh* m = shapes::createMeshFromResource(path_to_mesh, scale);
//   shapes::ShapeMsg mesh_msg;
//   shapes::constructMsgFromShape(m, mesh_msg);
//   delete m;

//   shape_msgs::Mesh mesh;
//   mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

//   collision_object.meshes.resize(1);
//   collision_object.mesh_poses.resize(1);
//   collision_object.meshes[0] = mesh;
//   collision_object.header.frame_id = reference_frame;

//   collision_object.mesh_poses[0] = pose;

//   collision_object.meshes.push_back(mesh);
//   collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
//   collision_object.operation = collision_object.ADD;

//   return collision_object;
// }

// moveit_msgs::CollisionObject toCollisionObject(const object_t& obj)
// {
//   //return toCollisionObject(obj.id(), obj.path_to_mesh(), obj.reference_frame(), obj.mesh_pose(), obj.scale());
//   return toCollisionObject(obj.id, obj.path_to_mesh(), obj.reference_frame(), obj.mesh_pose(), obj.scale());
// }

bool TFNamedObjectsManager::moveObjects(const std::map<std::string, geometry_msgs::Pose>& objs_poses_map, const std::map<std::string,std_msgs::ColorRGBA>& objs_colors_map, const double timeout_s, std::string& what)
{
  auto start_time = high_resolution_clock::now();

  std::vector<std::string> ids;
  ids.reserve(objs_poses_map.size());

  for (const auto &o : objs_poses_map)
    ids.push_back(o.first);

  std::map<std::string, moveit_msgs::CollisionObject> objs = planning_scene_interface_.getObjects(ids);
  double _timeout_s = timeout_s - duration_cast<seconds>(high_resolution_clock::now() - start_time).count();

  if(objs.empty())
  {
    what = what + "\nids don't match any object in the scene";
    return false;
  }

  std::vector<std_msgs::ColorRGBA> ccolors; ccolors.reserve(objs.size());
  std::vector<moveit_msgs::CollisionObject> cobjs; cobjs.reserve(objs.size());

  for(auto& [id, cobj] : objs)
  {
    cobj.pose = objs_poses_map.at(id);
    cobj.operation = moveit_msgs::CollisionObject::MOVE;

    cobj.meshes    .clear(); //remove the warnings
    cobj.planes    .clear(); //remove the warnings
    cobj.primitives.clear(); //remove the warnings

    cobjs.push_back(cobj);
    ccolors.push_back(objs_colors_map.at(id));
  }
  if (cobjs.size())
  {
    bool ret = applyAndCheck(cobjs, ccolors, _timeout_s, what);
    return ret;
  }
  return true;
}

bool TFNamedObjectsManager::moveNamedTFObjects(const std::map<std::string, geometry_msgs::Pose>& objs_poses_map,
                                               const std::map<std::string,std_msgs::ColorRGBA>& objs_colors_map,
                                               const double timeout_s, std::string& what)
{
  if(!moveObjects(objs_poses_map, objs_colors_map, timeout_s, what))
    return false;

  std::vector<std::string> ids;
  ids.reserve(objs_poses_map.size());

  for (const auto &o : objs_poses_map)
    ids.push_back(o.first);

  std::vector<std::string> move_tf;
  std::vector<TFPublisherThread::Ptr>::iterator it;
  for(const std::string& id : ids)
  {
    it = std::find_if(tf_publishers_.begin(), tf_publishers_.end(),
                      [&id](const TFPublisherThread::Ptr& tf_publisher) {return tf_publisher->tf_object_name() == id;});

    if(it != tf_publishers_.end())
    {
      move_tf.push_back((*it)->tf_object_name());
      (*it)->pose(objs_poses_map.at(id));
    }
  }

//  if(not are_tf_available(move_tf, timeout_s, what))
//  {
//    what =
//        "Timeout Expired. The TF " + to_string(move_tf) + " are not in the scene after " + std::to_string(timeout_s) + "sec.";
//    return false;
//  }
  return true;
}

/**
 * @brief
 *
 * @param tf_named_objects
 * @param timeout_s
 * @param what
 * @return true
 * @return false
 */

bool TFNamedObjectsManager::addNamedTFObjects(const tf_named_objects_t& tf_named_objects, double timeout_s, std::string& what)
{
  std_msgs::ColorRGBA color;
  color.r = 255;  color.g = 0;  color.b = 0;  color.a = 1;
  std::vector<std_msgs::ColorRGBA> colors(tf_named_objects.size(), color);

  return addNamedTFObjects(tf_named_objects,timeout_s,colors,what);
}
bool TFNamedObjectsManager::addNamedTFObjects(const tf_named_objects_t& tf_named_objects, double timeout_s,
                                              const std::vector<std_msgs::ColorRGBA>& colors, std::string& what)
{
  if (!tfListener_)
  {
    tfListener_.reset(new tf2_ros::TransformListener(tfBuffer_));
    ros::Duration(tfBuffer_.getCacheLength()).sleep();
  }

  ROS_INFO("[Add Named Object] Add %zu objects", tf_named_objects.size());
  double _timeout_s = timeout_s;
  auto start_time = high_resolution_clock::now();

  // =====================================
  // CHECK IF THE TF PARENTS DO EXIST
  // =====================================
  auto reference_frames = get_reference_frame_id(tf_named_objects);
  ROS_INFO("[Add Named Object] Check if the reference frames (%s) are in the list of TF published (timeout: %f)",
           to_string(reference_frames).c_str(), timeout_s);
  auto now = high_resolution_clock::now();
  if (!are_tf_available(reference_frames, timeout_s, what))
  {
    return false;
  }

  // =====================================
  // REMOVE ALREADY PRESENT OBJECTS
  // =====================================
  _timeout_s = timeout_s - duration_cast<seconds>(high_resolution_clock::now() - start_time).count();
  ROS_INFO("[Add Named Object] Remove objects if present (timeout: %f)", _timeout_s);
  if (!removeObjects(get_id(tf_named_objects), _timeout_s, what))
  {
    return false;
  }

  // =====================================
  // == REMOVE ALREADY EXISTING TF PUBLISHER OF THE OBJ TF FRAME
  // =====================================
  _timeout_s = timeout_s - duration_cast<seconds>(high_resolution_clock::now() - start_time).count();
  ROS_INFO("[Add Named Object] Remove TF publisher present (timeout: %f)", _timeout_s);
  objects_t objs;
  for (const auto& obj : tf_named_objects)
  {
    // Check If the TF OBJ is published by this library
    ROS_INFO("[Add Named Object] Check if the OBJ is already published by this library");
    auto it = std::find_if(tf_publishers_.begin(), tf_publishers_.end(), [&obj](const TFPublisherThread::Ptr& tf_pub) {
      return tf_pub->tf_object_name() == obj.tf_name();
    });
    if (it != tf_publishers_.end())
    {
      ROS_INFO("[Add Named Object] The OBJ is already published by this library!");
      ROS_INFO("[Add Named Object] Exit from the thread..");
      (*it)->exit();
      tf_publishers_.erase(it);
    }

    // Check If there are other TF publisher
    _timeout_s = timeout_s - duration_cast<seconds>(high_resolution_clock::now() - start_time).count();
    ROS_INFO("[Add Named Object] Check if other TF publisher may overlap our TF (timeout: %f)", _timeout_s);
    if (!are_tf_unavailable({ obj.tf_name() }, timeout_s, what))
    {
      return false;
    }

    objs.push_back(obj);
  }
  // =====================================

  // =====================================
  // ADD OBJECTS
  ROS_INFO("[Add Named Object] Add Objects (timeout: %f)", timeout_s);
  _timeout_s = timeout_s - duration_cast<seconds>(high_resolution_clock::now() - start_time).count();
  if (!addObjects(objs, _timeout_s, colors, what))
  {
    return false;
  }

  // =====================================
  // ADD TF PUBLISHER
  ROS_INFO("[Add Named Object] Add TF Publisher (timeout: %f)", timeout_s);
  for (const auto& tf_named_object : tf_named_objects)
  {
    // std::cout << tf_named_object.mesh_pose() << std::endl;
    TFPublisherThread::Ptr tf_pub(
          new TFPublisherThread(tf_named_object.tf_name(), tf_named_object.header.frame_id, tf_named_object.pose));
    tf_publishers_.push_back(tf_pub);
  }

  // =====================================
  // CHECK IF THE OBJ TF  DO EXIST
  ROS_INFO("[Add Named Object] Check if TF are published (timeout: %f)", timeout_s);
  _timeout_s = timeout_s - duration_cast<seconds>(high_resolution_clock::now() - start_time).count();
  if (!are_tf_available(get_id(tf_named_objects), _timeout_s, what))
  {
    return false;
  }

  return true;
}

bool TFNamedObjectsManager::addObjects(const objects_t& objs, double timeout_s, std::string& what)
{
  std_msgs::ColorRGBA color;
  color.r = 255;  color.g = 255;  color.b = 255;  color.a = 1;
  std::vector<std_msgs::ColorRGBA> colors(objs.size(), color);

  return addObjects(objs,timeout_s,colors,what);
}

bool TFNamedObjectsManager::addObjects(const objects_t& objs, double timeout_s, const std::vector<std_msgs::ColorRGBA>& colors, std::string& what)
{
  if (!tfListener_)
  {
    tfListener_.reset(new tf2_ros::TransformListener(tfBuffer_));
    ros::Duration(tfBuffer_.getCacheLength()).sleep();
  }

  auto start_time = high_resolution_clock::now();
  ROS_INFO("[Add Object] Add %zu objects", objs.size());
  ROS_INFO("[Add Object] Check if TF reference frame is published");
  auto reference_frames = get_reference_frame_id(objs);
  if (!are_tf_available(reference_frames, timeout_s, what))
  {
    what = "Timeout Expired. Some of the objects " + to_string(objs) +
        " refer to frames that are not in the list of the tf  (lookup timeout: " + std::to_string(timeout_s) +
        "sec): " + what;
    return false;
  }
  double _timeout_s = timeout_s - duration_cast<seconds>(high_resolution_clock::now() - start_time).count();

  std::vector<moveit_msgs::CollisionObject> cobjs;
  //  std::vector<std::string> v = planning_scene_interface_.getKnownObjectNames();
  std::vector<std_msgs::ColorRGBA> ccolors;

  ROS_INFO("[Add Object] Fill Collision Object msg");
  for (size_t i = 0; i < objs.size(); i++)
  {
    if (objs.at(i).mesh_poses.size() == 0 && objs.at(i).primitive_poses.size() == 0 &&
        objs.at(i).subframe_poses.size() == 0 && objs.at(i).plane_poses.size() == 0)
    {
      continue;
    }
    // cobjs.push_back(toCollisionObject(objs.at(i).id(), objs.at(i).path_to_mesh(), objs.at(i).reference_frame(),
    //                                   objs.at(i).mesh_pose(), objs.at(i).scale()));

    cobjs.push_back(objs.at(i));
    ccolors.push_back(colors.at(i));
  }
  if (cobjs.size())
  {
    ROS_INFO("[Add Object] Apply and Check %zu", cobjs.size());
    bool ret = applyAndCheck(cobjs, ccolors, _timeout_s, what);
    ROS_INFO("[Add Object] Apply and Check returned %s", (ret?"OK":"FAILED"));
    return ret;
  }
  return true;
}

bool TFNamedObjectsManager::check(const std::vector<std::string>& tf_names, const double& timeout_s,
                                  bool check_if_available, std::string& what)
{
  if (!tfListener_)
  {
    tfListener_.reset(new tf2_ros::TransformListener(tfBuffer_));
    ros::Duration(tfBuffer_.getCacheLength()).sleep();
  }

  auto start_time = high_resolution_clock::now();

  std::vector<std::string> _tf_names = tf_names;
  do
  {
    std::string cached_frames = tfBuffer_.allFramesAsYAML();
    YAML::Node config = YAML::Load(cached_frames);
    std::vector<std::string> _frames;
    std::vector<std::string> _parent_frames;
    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
    {
      _frames.push_back(it->first.as<std::string>());
      _parent_frames.push_back(it->second["parent"].as<std::string>());
    }

    bool frame_update_available = std::find(_frames.begin(), _frames.end(), _tf_names.back().c_str()) != _frames.end();
    bool parent_frame_update_available =
        std::find(_parent_frames.begin(), _parent_frames.end(), _tf_names.back().c_str()) != _parent_frames.end();

    if (frame_update_available)  // check if updated!
    {
      std::string parent = config[_tf_names.back().c_str()]["parent"].as<std::string>();
      std::string tf_err;
      double _lasting_time = (timeout_s-duration_cast<seconds>(high_resolution_clock::now() - start_time).count());
      auto _timeout_s = ros::Duration((_lasting_time < 1.0 ? 1.0 : _lasting_time));
      frame_update_available =
          tfBuffer_.canTransform(_tf_names.back(), parent, ros::Time::now(), _timeout_s, &tf_err);
    }
    else
    {
      frame_update_available = parent_frame_update_available;
    }

    if ((frame_update_available && check_if_available) || (!frame_update_available && !check_if_available))
    {
      _tf_names.pop_back();
    }

    if (_tf_names.size() == 0)
    {
      break;
    }
    else
    {
      if (duration_cast<seconds>(high_resolution_clock::now() - start_time).count() > timeout_s)
      {
        what = "Timeout Expired. Checked the status of the frames: " + to_string(tf_names) +
            ". The actual available frames are " + to_string(_frames) + " (timeout: " + std::to_string(timeout_s) +
            "sec)";
        return false;
      }
    }
  } while (true);

  return true;
}

bool TFNamedObjectsManager::are_tf_available(const std::vector<std::string>& tf_names, const double& timeout_s,
                                             std::string& what)
{
  return check(tf_names, timeout_s, true, what);
}

bool TFNamedObjectsManager::are_tf_unavailable(const std::vector<std::string>& tf_names, const double& timeout_s,
                                               std::string& what)
{
  return check(tf_names, timeout_s, false, what);
}

bool TFNamedObjectsManager::removeObjects(const std::vector<std::string>& ids, const double timeout_s,
                                          std::string& what)
{
  std::vector<std::string> vv = planning_scene_interface_.getKnownObjectNames();
  std::vector<std::string> _ids;
  for (const auto& v : vv)
  {
    if (std::find(ids.begin(), ids.end(), v) != ids.end())
    {
      _ids.push_back(v);
    }
  }
  planning_scene_interface_.removeCollisionObjects(_ids);

  return waitUntil(ids, { TFNamedObjectsManager::ObjectState::DETACHED, TFNamedObjectsManager::ObjectState::UNKNOWN },
                   timeout_s, what);
}

bool TFNamedObjectsManager::removeNamedObjects(const std::vector<std::string>& ids, const double timeout_s,
                                               std::string& what)
{
  if (!removeObjects(ids, timeout_s, what))
  {
    return false;
  }

  std::vector<std::string> remove_tf;
  std::vector<TFPublisherThread::Ptr>::iterator it;
  for(const std::string& id : ids)
  {
    it = std::find_if(tf_publishers_.begin(), tf_publishers_.end(),
                      [&id](const TFPublisherThread::Ptr& tf_publisher) {return tf_publisher->tf_object_name() == id;});

    if(it != tf_publishers_.end())
    {
      remove_tf.push_back((*it)->tf_object_name());
      (*it)->exit();
      tf_publishers_.erase(it);
    }
  }

  if (are_tf_available(remove_tf, timeout_s, what))
  {
    what =
        "Timeout Expired. The TF " + to_string(remove_tf) + " are still the scene after " + std::to_string(timeout_s) + "sec.";
    return false;
  }
  return true;
}

bool TFNamedObjectsManager::resetScene(const double timeout_s, std::string& what)
{
  std::vector<std::string> vv = planning_scene_interface_.getKnownObjectNames();
  return removeNamedObjects(vv, timeout_s, what);
}

bool TFNamedObjectsManager::waitUntil(const std::vector<std::string>& object_names,
                                      const std::vector<TFNamedObjectsManager::ObjectState>& checks, double timeout,
                                      std::string& what)
{
  auto start_time = high_resolution_clock::now();

  std::vector<bool> oks(object_names.size(), false);
  while (duration_cast<seconds>(high_resolution_clock::now() - start_time).count() < timeout)
  {
    for (const auto& check : checks)
    {
      if (check == ObjectState::ATTACHED or check == ObjectState::DETACHED)
      {
        auto const cobjs = planning_scene_interface_.getObjects(object_names);
        for (size_t i = 0; i < object_names.size(); i++)
        {
          const auto& object_name = object_names.at(i);
          if (oks.at(i))
          {
            continue;
          }
          bool attached = (cobjs.find(object_name) != cobjs.end());
          oks.at(i) = ((check == ObjectState::ATTACHED && attached) || (check == ObjectState::DETACHED && !attached));
        }
      }
      else if (check == ObjectState::KNOWN or check == ObjectState::UNKNOWN)
      {
        auto const obj_names = planning_scene_interface_.getKnownObjectNames();
        for (size_t i = 0; i < object_names.size(); i++)
        {
          const auto& object_name = object_names.at(i);
          if (oks.at(i))
          {
            continue;
          }
          bool known = (std::find(obj_names.begin(), obj_names.end(), object_name) != obj_names.begin());
          oks.at(i) = ((check == ObjectState::KNOWN && known) || (check == ObjectState::UNKNOWN && !known));
        }
      }
    }

    if (std::all_of(oks.begin(), oks.end(), [](bool v) { return v; }))
    {
      return true;
    }

    std::this_thread::sleep_for(25ms);
  }

  what = "Timeout Expired. The objects " + to_string(object_names) + " are not yet in the scene after " +
      std::to_string(timeout) + "sec.";
  return false;
}

bool TFNamedObjectsManager::applyAndCheck(const std::vector<moveit_msgs::CollisionObject>& cov,
                                          const std::vector<std_msgs::ColorRGBA>& colors, const double& timeout_s,
                                          std::string& what)
{
  if (cov.size() == 0)
  {
    what = "No objects to be added to the scene or moved";
    return false;
  }
  std::vector<std::string> object_names;
  bool cumulative_check = true;
  std::vector<uint8_t> operations;
  for (const moveit_msgs::CollisionObject& co : cov)
  {
    operations.push_back(co.operation);
  }
  cumulative_check =
      operations.size() == 1 ? true : std::equal(operations.begin() + 1, operations.end(), operations.begin());

  for (size_t i = 0; i < cov.size(); i++)
  {
    planning_scene_interface_.applyCollisionObject(cov.at(i), colors.at(i));
    object_names.push_back(cov.at(i).id);
  }

  bool ret = true;
  for (size_t i = 0; i < operations.size(); i++)
  {
    std::vector<TFNamedObjectsManager::ObjectState> st =
        operations.at(i) != moveit_msgs::CollisionObject::REMOVE ?
          std::vector<TFNamedObjectsManager::ObjectState>{ TFNamedObjectsManager::ObjectState::KNOWN,
          TFNamedObjectsManager::ObjectState::ATTACHED } :
          std::vector<TFNamedObjectsManager::ObjectState>{ TFNamedObjectsManager::ObjectState::UNKNOWN,
          TFNamedObjectsManager::ObjectState::DETACHED };

    std::vector<std::string> _object_names =
        cumulative_check ? object_names : std::vector<std::string>{ object_names.at(i) };

    ret = waitUntil(_object_names, st, timeout_s, what);
    if (!ret)
    {
      return false;
    }
    if (cumulative_check)
    {
      break;
    }
  }
  return ret;
}

TFNamedObjectsManager::TFPublisherThread::TFPublisherThread(const std::string& tf_obj_frame,
                                                            const std::string& tf_reference_frame,
                                                            const geometry_msgs::Pose& pose)
  : tf_obj_frame_(tf_obj_frame)
  , tf_reference_frame_(tf_reference_frame)
  , pose_(pose)
  , future_obj_(exit_signal_.get_future())
  , thread_(&TFPublisherThread::thread_function, this)
{
}
bool TFNamedObjectsManager::TFPublisherThread::exit()
{
  if (thread_.joinable())
  {
    exit_signal_.set_value();
    ROS_ERROR("..... Waiting for joining the thread that publish the TF '%s'", tf_obj_frame_.c_str());
    thread_.join();
  }
  return true;
}

const std::string& TFNamedObjectsManager::TFPublisherThread::tf_object_name() const
{
  return tf_obj_frame_;
}

const std::string& TFNamedObjectsManager::TFPublisherThread::tf_reference_name() const
{
  return tf_obj_frame_;
}

const geometry_msgs::Pose& TFNamedObjectsManager::TFPublisherThread::pose() const
{
  return pose_;
}
void TFNamedObjectsManager::TFPublisherThread::pose(const geometry_msgs::Pose& pose)
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    pose_ = std::move(pose);
  }
}

void TFNamedObjectsManager::TFPublisherThread::thread_function()
{
  ROS_INFO("Starting to Pulish of '%s'", tf_obj_frame_.c_str());
  tf2_ros::TransformBroadcaster tf_broadcaster;
  while (future_obj_.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
  {
    geometry_msgs::TransformStamped tfs;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      tfs.header.stamp = ros::Time::now();
      tfs.header.frame_id = tf_reference_frame_;
      tfs.child_frame_id = tf_obj_frame_;
      tfs.transform.translation.x = pose_.position.x;
      tfs.transform.translation.y = pose_.position.y;
      tfs.transform.translation.z = pose_.position.z;
      tfs.transform.rotation.x = pose_.orientation.x;
      tfs.transform.rotation.y = pose_.orientation.y;
      tfs.transform.rotation.z = pose_.orientation.z;
      tfs.transform.rotation.w = pose_.orientation.w;
    }
    tf_broadcaster.sendTransform(tfs);
    ros::spinOnce();
    std::this_thread::sleep_for(100ms);
  }
  ROS_INFO("Exiting from the TF Pulisher of '%s'", tf_obj_frame_.c_str());
  std::this_thread::sleep_for(500ms);
}

}  // namespace cnr_tf_named_object_loader
