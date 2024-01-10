#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/service.h>

#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit_object_loader/moveit_object_loader.h>


std::shared_ptr<moveit_object_loader::TFNamedObjectsManager> tf_named_objects_;

bool add(moveit_msgs::ApplyPlanningScene::Request  &req,
         moveit_msgs::ApplyPlanningScene::Response &res)
{
  ROS_INFO("=================================================================");
  ROS_INFO("== LOAD TF NAMED OBJECT! REQUEST RECEIVED!                    ===");
  ROS_INFO("=================================================================");
  std::string what;
  double timeout_s = 10.0;
  ros::param::get("~/timeout", timeout_s);

  moveit_object_loader::tf_named_objects_t tf_named_objs;
  for(const auto & obj : req.scene.world.collision_objects)
  {
    std::cout << obj << std::endl;
    tf_named_objs.push_back(moveit_object_loader::toCollisionObject(obj));
  }

  if(!tf_named_objects_->addNamedTFObjects(tf_named_objs, timeout_s, what))
  {
    ROS_ERROR("Error in adding the object to the scene: '%s'", what.c_str());
    res.success = false;
    return true;
  }
  ROS_INFO("=================================================================");
  return (res.success = true);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_object_loader");
  ros::NodeHandle n;

  tf_named_objects_.reset(new moveit_object_loader::TFNamedObjectsManager());

  ros::ServiceServer service = n.advertiseService("load_tf_named_object", add);
  ROS_INFO("Ready to add objects to the scene");
  ros::spin();

  return 0;
}