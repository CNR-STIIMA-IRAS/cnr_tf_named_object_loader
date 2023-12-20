#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <geometry_msgs/TransformStamped.h>

#include <shape_msgs/Mesh.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_object_loader/moveit_object_loader.h>

#include <chrono>
#include <cstdio>
#include <thread>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/init.h"
#include "ros/time.h"
#include <std_msgs/ColorRGBA.h>

using namespace std::chrono;
using namespace std::chrono_literals;

TFNamedObjectsManager::TFNamedObjectsManager() : tfListener_(tfBuffer_){ /* nothing to do so far */ };

std::string to_string(const objects_t& vv)
{
  std::string ret = "[";
  for (const auto& v : vv)
    ret += "{id:" + v.id() + ", reference_frame:" + v.reference_frame() + "}, ";
  return ret + "]";
}

std::string to_string(const tf_named_objects_t& vv)
{
  std::string ret = "[";
  for (const auto& v : vv)
    ret += "{id:" + v.id() + ", " + " frame:" + v.tf_name() + ", reference_frame:" + v.reference_frame() + "}, ";
  return ret + "]";
}

std::string to_string(const std::vector<std::string>& vv)
{
  std::string ret = "[";
  for (const auto& v : vv)
    ret += v + ",";
  return ret + "]";
}

std::vector<std::string> get_id(const objects_t& vv)
{
  std::vector<std::string> ret;
  for (const auto& v : vv)
    ret.push_back(v.id());
  return ret;
}

std::vector<std::string> get_id(const tf_named_objects_t& vv)
{
  std::vector<std::string> ret;
  for (const auto& v : vv)
    ret.push_back(v.id());
  return ret;
}

std::vector<std::string> get_frame_id(const tf_named_objects_t& vv)
{
  std::vector<std::string> ret;
  for (const auto& v : vv)
    ret.push_back(v.tf_name());
  return ret;
}

bool TFNamedObjectsManager::waitUntil(const std::vector<std::string>& object_names,
                                    const std::vector<TFNamedObjectsManager::ObjectState>& checks, double timeout,
                                    std::string& what)
{
  auto start_time = high_resolution_clock::now();

  while (duration_cast<seconds>(high_resolution_clock::now() - start_time).count() < timeout)
  {
    bool ok = true;
    for (const auto& check : checks)
    {
      if (check == ObjectState::ATTACHED or check == ObjectState::DETACHED)
      {
        auto const cobjs = planning_scene_interface_.getObjects(object_names);
        bool attached = true;
        for (const auto& object_name : object_names)
        {
          attached &= (cobjs.find(object_name) != cobjs.end());
        }
        ok &= ((check == ObjectState::ATTACHED && attached) || (check == ObjectState::DETACHED && !attached));
      }
      else if (check == ObjectState::KNOWN or check == ObjectState::UNKNOWN)
      {
        auto const obj_names = planning_scene_interface_.getKnownObjectNames();
        bool known = true;
        for (const auto& object_name : object_names)
        {
          known &= (std::find(obj_names.begin(), obj_names.end(), object_name) != obj_names.begin());
        }
        ok &= ((check == ObjectState::KNOWN && known) || (check == ObjectState::UNKNOWN && !known));
      }
    }

    if (ok)
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

    ret = waitUntil(object_names, st, timeout_s, what);
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

moveit_msgs::CollisionObject TFNamedObjectsManager::toCollisionObject(const std::string& collisionObjID,
                                                                    const std::string& path_to_mesh,
                                                                    const std::string& reference_frame,
                                                                    const geometry_msgs::Pose& pose,
                                                                    const Eigen::Vector3d scale)
{
  std::shared_ptr<moveit_msgs::CollisionObject> collision_object(new moveit_msgs::CollisionObject());

  collision_object->id = collisionObjID;

  shapes::Mesh* m = shapes::createMeshFromResource(path_to_mesh, scale);
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  delete m;

  shape_msgs::Mesh mesh;
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  collision_object->meshes.resize(1);
  collision_object->mesh_poses.resize(1);
  collision_object->meshes[0] = mesh;
  collision_object->header.frame_id = reference_frame;

  collision_object->mesh_poses[0] = pose;

  collision_object->meshes.push_back(mesh);
  collision_object->mesh_poses.push_back(collision_object->mesh_poses[0]);
  collision_object->operation = collision_object->ADD;

  return *collision_object;
}

bool TFNamedObjectsManager::addObjects(const objects_t& objs, double timeout_s, std::string& what)
{
  objects_t _objs;
  auto start_time = high_resolution_clock::now();
  while (_objs.size() == objs.size())
  {
    geometry_msgs::TransformStamped ts;
    try
    {
      ts = tfBuffer_.lookupTransform("world", objs.at(_objs.size()).reference_frame(), ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      continue;
    }
    _objs.push_back(objs.at(_objs.size()));
    if (duration_cast<seconds>(high_resolution_clock::now() - start_time).count() > timeout_s)
    {
      what = "Timeout Expired. Some of the objects " + to_string(objs) +
             " refer to frames that are not in the list of the tf  (lookup timeout: " + std::to_string(timeout_s) +
             "sec)";
      return false;
    }
  }
  double _timeout_s = timeout_s - duration_cast<seconds>(high_resolution_clock::now() - start_time).count();

  std::vector<moveit_msgs::CollisionObject> cobjs;
  std::vector<std_msgs::ColorRGBA> colors;
  std::vector<std::string> v = planning_scene_interface_.getKnownObjectNames();

  for (size_t i = 0; i < objs.size(); i++)
  {
    cobjs.push_back(toCollisionObject(objs.at(i).id(), objs.at(i).path_to_mesh(), objs.at(i).reference_frame(),
                                      objs.at(i).pose(), objs.at(i).scale()));

    std_msgs::ColorRGBA color;
    color.r = 255;
    color.g = 255;
    color.b = 255;
    color.a = 1;
    colors.push_back(color);
  }
  return applyAndCheck(cobjs, colors, _timeout_s, what);
}

bool TFNamedObjectsManager::are_tf_available(const std::vector<std::string>& tf_names, const double& timeout_s,
                                           std::string& what)
{
  auto start_time = high_resolution_clock::now();
  std::vector<std::string> ok;
  while (ok.size() < tf_names.size())
  {
    geometry_msgs::TransformStamped ts;
    try
    {
      ts = tfBuffer_.lookupTransform("world", tf_names.at(ok.size()), ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      continue;
    }
    ok.push_back(tf_names.at(ok.size()));
    if (duration_cast<seconds>(high_resolution_clock::now() - start_time).count() > timeout_s)
    {
      what = "Timeout Expired. Asked for " + to_string(tf_names) + ", but only " + to_string(ok) +
             " are publishedafter " + std::to_string(timeout_s) + "sec";
      return false;
    }
  }
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
bool TFNamedObjectsManager::addNamedTFObjects(const tf_named_objects_t& tf_named_objects, double timeout_s,
                                            std::string& what)
{
  auto start_time = high_resolution_clock::now();

  // =====================================
  // CHECK IF THE TF PARENTS DO EXIST
  // =====================================
  auto now = high_resolution_clock::now();
  if (!are_tf_available(get_frame_id(tf_named_objects), timeout_s, what))
  {
    return false;
  }

  // =====================================
  // REMOVE ALREADY PRESENT OBJECTS
  // =====================================
  if (!removeObjects(get_id(tf_named_objects), timeout_s - (high_resolution_clock::now() - start_time).count(), what))
  {
    return false;
  }

  // =====================================
  // == REMOVE ALREADY EXISTING TF PUBLISHER OF THE OBJ TF FRAME
  // =====================================
  objects_t objs;
  for (const auto& obj : tf_named_objects)
  {
    // Check If the TF OBJ is published by this library
    auto it = std::find_if(tf_publishers_.begin(), tf_publishers_.end(),
                           [&obj](const TFPublisherThread::Ptr& tf_pub) { return tf_pub->tf_object_name() == obj.tf_name(); });
    if (it != tf_publishers_.end())
    {
      (*it)->exit();
      tf_publishers_.erase(it);
    }

    // Check If there are other TF publisher
    geometry_msgs::TransformStamped ts;
    try
    {
      ts = tfBuffer_.lookupTransform("world", obj.tf_name(), ros::Time(0));
      what = "A TF publisher already exists for this TF name: " + obj.tf_name() + ". Abort";
      return false;
    }
    catch (tf2::TransformException& ex)
    {
      // OK none TF with the same name exists
      continue;
    }
    objs.push_back(obj);
  }
  // =====================================

  // =====================================
  // ADD OBJECTS
  if (!addObjects(objs, timeout_s - (high_resolution_clock::now() - start_time).count(), what))
  {
    return false;
  }

  // =====================================
  // ADD TF PUBLISHER
  for (const auto& tf_named_object : tf_named_objects)
  {
    TFPublisherThread::Ptr tf_pub(new TFPublisherThread(tf_named_object.tf_name(), tf_named_object.reference_frame(), tf_named_object.pose()));
    tf_publishers_.push_back(tf_pub);
  }

  // =====================================
  // CHECK IF THE OBJ TF  DO EXIST
  if (!are_tf_available(get_id(tf_named_objects), timeout_s - (high_resolution_clock::now() - start_time).count(),
                        what))
  {
    return false;
  }

  return true;
}

bool TFNamedObjectsManager::removeObjects(const std::vector<std::string>& ids, const double timeout_s, std::string& what)
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


bool TFNamedObjectsManager::removeNamedObjects(const std::vector<std::string>& ids, const double timeout_s, std::string& what)
{
  if(!removeObjects(ids, timeout_s, what))
  {
    return false;

  }
  std::vector<std::string> vv;
  for(const auto & p  : tf_publishers_)
  {
    vv.push_back(p->tf_object_name());
  }

  for(auto & tf_publisher : tf_publishers_)
  {
    tf_publisher->exit();
  }
  tf_publishers_.clear();

  if(are_tf_available(vv, timeout_s, what))
  {
    what = "Timeout Expired. The TF " + to_string(vv) + " are still the scene after " + std::to_string(timeout_s) + "sec.";
    return false;
  }
  return true;
}


bool TFNamedObjectsManager::resetScene(const double timeout_s, std::string& what)
{
  std::vector<std::string> vv = planning_scene_interface_.getKnownObjectNames();
  return removeNamedObjects(vv, timeout_s, what);
}

const std::string& object_t::id() const
{
  return std::get<0>(*this);
}

std::string& object_t::id()
{
  return std::get<0>(*this);
}
const std::string& object_t::path_to_mesh() const
{
  return std::get<1>(*this);
}
std::string& object_t::path_to_mesh()
{
  return std::get<1>(*this);
}
const std::string& object_t::reference_frame() const
{
  return std::get<2>(*this);
}
std::string& object_t::reference_frame()
{
  return std::get<2>(*this);
}
const geometry_msgs::Pose& object_t::pose() const
{
  return std::get<3>(*this);
}
geometry_msgs::Pose& object_t::pose()
{
  return std::get<3>(*this);
}

const Eigen::Vector3d& object_t::scale() const
{
  return scale_;
}

Eigen::Vector3d& object_t::scale()
{
  return scale_;
}

const std::string& tf_named_object_t::tf_name() const
{
  return tf_name_;
}
std::string& tf_named_object_t::tf_name()
{
  return tf_name_;
}
TFNamedObjectsManager::TFPublisherThread::TFPublisherThread(const std::string& tf_obj_frame, const std::string& tf_reference_frame, const geometry_msgs::Pose& pose)
  : tf_obj_frame_(tf_obj_frame)
  , tf_reference_frame_(tf_reference_frame)
  , pose_(pose)
  , future_obj_(exit_signal_.get_future())
  , thread_(&TFPublisherThread::thread_function, this)
{
}
bool TFNamedObjectsManager::TFPublisherThread::exit()
{
  exit_signal_.set_value();
  if (thread_.joinable())
  {
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

void TFNamedObjectsManager::TFPublisherThread::thread_function()
{
  static tf2_ros::TransformBroadcaster tf_broadcaster;
  while (future_obj_.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
  {
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = tf_reference_frame_;
    tfs.child_frame_id = tf_obj_frame_;
    tfs.transform.translation.x = pose_.position.x;
    tfs.transform.translation.y = pose_.position.y;
    tfs.transform.translation.z = pose_.position.z;
    tfs.transform.rotation.x = pose_.orientation.x;
    tfs.transform.rotation.y = pose_.orientation.x;
    tfs.transform.rotation.z = pose_.orientation.z;
    tfs.transform.rotation.w = pose_.orientation.x;
    tf_broadcaster.sendTransform(tfs);
    ros::spinOnce();
  }
}
