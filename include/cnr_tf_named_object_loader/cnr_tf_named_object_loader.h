#ifndef AWARE_DEXTER_ROS_INTERFACE__SCENE_OBJECTS_MANAGER__H
#define AWARE_DEXTER_ROS_INTERFACE__SCENE_OBJECTS_MANAGER__H

#include <future>
#include <memory>
#include <thread>

#include <Eigen/Core>
#include "geometry_msgs/PoseStamped.h"
#include <Eigen/src/Core/Matrix.h>

#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

namespace cnr_tf_named_object_loader 
{ 

using object_t = moveit_msgs::CollisionObject;

// struct object_t : std::tuple<std::string, std::string, std::string, geometry_msgs::Pose>
// {
// private: 
//   Eigen::Vector3d scale_;
// public:
//   const std::string& id() const;
//   std::string& id();

//   const std::string& reference_frame() const;
//   std::string& reference_frame();

//   const std::string& path_to_mesh() const;
//   std::string& path_to_mesh();

//   const geometry_msgs::Pose& mesh_pose() const;
//   geometry_msgs::Pose& mesh_pose();

//   const Eigen::Vector3d& scale() const;
//   Eigen::Vector3d& scale();
// };

struct tf_named_object_t : object_t
{
private:
  std::string tf_name_;
public:
  tf_named_object_t() = default;
  tf_named_object_t(const object_t& obj);
  const std::string& tf_name() const;
  std::string& tf_name();
  tf_named_object_t& operator=(const object_t& obj);
};



using objects_t = std::vector<object_t>;
using tf_named_objects_t = std::vector<tf_named_object_t>;

std::vector<std::string> get_id(const objects_t& vv);
std::vector<std::string> get_id(const tf_named_objects_t& vv);
std::vector<std::string> get_frame_id(const tf_named_objects_t& vv);
std::vector<std::string> get_reference_frame_id(const tf_named_objects_t& vv);
std::vector<std::string> get_reference_frame_id(const objects_t& vv);

class TFNamedObjectsManager
{
public:
  explicit TFNamedObjectsManager();
  ~TFNamedObjectsManager() = default;

  TFNamedObjectsManager(const TFNamedObjectsManager&) = delete;
  TFNamedObjectsManager(TFNamedObjectsManager&&) = delete;

  bool addObjects(const objects_t& objs, double timeout_s, std::string& what);
  bool addObjects(const objects_t& objs, double timeout_s, const std::vector<std_msgs::ColorRGBA>& colors, std::string& what);
  bool addNamedTFObjects(const tf_named_objects_t& objs, double timeout_s, std::string& what);
  bool addNamedTFObjects(const tf_named_objects_t& objs, double timeout_s, const std::vector<std_msgs::ColorRGBA>& colors, std::string& what);

  bool  moveObjects(const std::map<std::string, geometry_msgs::Pose> &objs_poses_map,
                    const std::map<std::string,std_msgs::ColorRGBA>& objs_colors_map,
                    const double timeout_s, std::string& what);
  bool moveNamedTFObjects(const std::map<std::string, geometry_msgs::Pose> &objs_poses_map,
                          const std::map<std::string, std_msgs::ColorRGBA> &objs_colors_map,
                          const double timeout_s, std::string& what);
  
  bool removeObjects(const std::vector<std::string>& ids, const double timeout_s, std::string& what);
  bool removeNamedObjects(const std::vector<std::string>& ids, const double timeout_s, std::string& what);
  bool resetScene(const double timeout_s, std::string& what);

  bool are_tf_available(const std::vector<std::string>& tf_names, const double& timeout_s, std::string& what);
  bool are_tf_unavailable(const std::vector<std::string>& tf_names, const double& timeout_s, std::string& what);

protected:
  enum class ObjectState
  {
    KNOWN,
    UNKNOWN,
    ATTACHED,
    DETACHED
  };

  /*
   * The `moveit::planning_interface::PlanningSceneInterface` has been removed because it does not estabilish persistent connections with
   * the `get_planning_scene` and `apply_planning_scene` services. This lack of persistent connections causes significant latency in service calls,
   * resulting in a substantial slowdown of all operations. To address this issue, the core functions of the `planning_scene_interface` have been re-implemented here,
   * utilizing persistent connections between clients and servers. This enhancement provides a 50-100x speedup in performance.
  */
//  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  ros::NodeHandle node_handle_;
  ros::ServiceClient planning_scene_service_;
  ros::ServiceClient apply_planning_scene_service_;
  ros::Publisher planning_scene_diff_publisher_;

  std::vector<std::string> getKnownObjectNames(bool with_type = false);
  std::map<std::string, moveit_msgs::CollisionObject> getObjects(const std::vector<std::string>& object_ids);
  bool applyCollisionObjects(const std::vector<moveit_msgs::CollisionObject>& collision_objects,
                                                     const std::vector<moveit_msgs::ObjectColor>& object_colors);
  bool applyPlanningScene(const moveit_msgs::PlanningScene& planning_scene);
  void removeCollisionObjects(const std::vector<std::string>& object_ids) const;


  class TFPublisherThread
  {
    const std::string tf_obj_frame_;
    const std::string tf_reference_frame_;
    geometry_msgs::Pose pose_;
    std::promise<void> exit_signal_;
    std::future<void> future_obj_;
    std::thread thread_;
    std::mutex mtx_;

    void thread_function( );

  public:
    using Ptr = std::shared_ptr<TFPublisherThread>;
    TFPublisherThread() = delete;
    TFPublisherThread(const TFPublisherThread& ) = delete;
    TFPublisherThread(TFPublisherThread&& ) = delete;
    ~TFPublisherThread() = default;
    explicit TFPublisherThread(const std::string& tf_obj_frame, const std::string& tf_reference_frame, const geometry_msgs::Pose& pose);
    bool exit();
    const std::string& tf_object_name() const;
    const std::string& tf_reference_name() const;
    const geometry_msgs::Pose& pose() const;
    void pose(const geometry_msgs::Pose &pose);

  };

  std::vector<TFPublisherThread::Ptr> tf_publishers_;

  tf2_ros::Buffer tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  
  bool applyAndCheck(const std::vector<moveit_msgs::CollisionObject>& cov,
                     const std::vector<std_msgs::ColorRGBA>& colors, const double& timeout_s, std::string& what);

  bool waitUntil(const std::vector<std::string>& object_names, const std::vector<ObjectState>& checks, double timeout,
                 std::string& what);

  bool check(const std::vector<std::string>& tf_names, const double& timeout_s, bool check_if_available, std::string& what);
};

// moveit_msgs::CollisionObject toCollisionObject(const std::string& collisionObjID, const std::string& path_to_mesh,
//                                                const std::string& reference_frame, const geometry_msgs::Pose& pose,
//                                                const Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1));

moveit_msgs::CollisionObject toCollisionObject(const object_t& obj);
object_t toCollisionObject(const moveit_msgs::CollisionObject& obj);

}
#endif  // AWARE_DEXTER_ROS_INTERFACE__SCENE_OBJECTS_MANAGER__H
