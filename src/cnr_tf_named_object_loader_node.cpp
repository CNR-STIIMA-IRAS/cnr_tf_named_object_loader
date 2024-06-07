#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/service.h>

#include <moveit_msgs/ApplyPlanningScene.h>

#include <cnr_tf_named_object_loader/cnr_tf_named_object_loader.h>


std::shared_ptr<cnr_tf_named_object_loader::TFNamedObjectsManager> tf_named_objects_;

bool add(moveit_msgs::ApplyPlanningScene::Request  &req,
         moveit_msgs::ApplyPlanningScene::Response &res)
{
  ROS_INFO("=================================================================");
  ROS_INFO("== LOAD TF NAMED OBJECT! REQUEST RECEIVED!                    ===");
  ROS_INFO("=================================================================");
  std::string what;
  double timeout_s = 10.0;
  ros::param::get("~/timeout", timeout_s);

  cnr_tf_named_object_loader::tf_named_objects_t tf_named_objs;
  for(const auto & obj : req.scene.world.collision_objects)
  {
    std::cout << obj << std::endl;
    tf_named_objs.push_back(cnr_tf_named_object_loader::toCollisionObject(obj));
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

bool remove(moveit_msgs::ApplyPlanningScene::Request  &req,
         moveit_msgs::ApplyPlanningScene::Response &res);

bool move(moveit_msgs::ApplyPlanningScene::Request  &req,
         moveit_msgs::ApplyPlanningScene::Response &res);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cnr_tf_named_object_loader");
  ros::NodeHandle n;

  tf_named_objects_.reset(new cnr_tf_named_object_loader::TFNamedObjectsManager());

  ros::ServiceServer add_service = n.advertiseService("load_tf_named_object", add);
  ROS_INFO("Ready to add objects to the scene");
  ros::ServiceServer move_service = n.advertiseService("move_tf_named_object", move);
  ROS_INFO("Ready to move objects to the scene");
  ros::ServiceServer remove_service = n.advertiseService("unload_tf_named_object", remove);
  ROS_INFO("Ready to remove objects to the scene");

  ros::spin();

  return 0;
}
