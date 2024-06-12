#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

#include <cnr_tf_named_object_loader/cnr_tf_named_object_loader.h>

std::shared_ptr<rclcpp::Node> node_;
std::shared_ptr<cnr_tf_named_object_loader::TFNamedObjectsManager> tf_named_objects_;

void add(const std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Request> req,
         std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Response> res)
{
  RCLCPP_INFO(node_->get_logger(),"=================================================================");
  RCLCPP_INFO(node_->get_logger(),"== LOAD TF NAMED OBJECT! REQUEST RECEIVED!                    ===");
  RCLCPP_INFO(node_->get_logger(),"=================================================================");
  std::string what;
  double timeout_s;
  node_->declare_parameter("~/timeout", 10.0);
  node_->get_parameter("~/timeout", timeout_s);

  cnr_tf_named_object_loader::tf_named_objects_t tf_named_objs;
  for(const auto & obj : req->scene.world.collision_objects)
  {
    std::cout << obj.id << std::endl;
    tf_named_objs.push_back(cnr_tf_named_object_loader::toCollisionObject(obj));
  }

  if(!tf_named_objects_->addNamedTFObjects(tf_named_objs, timeout_s, what))
  {
    RCLCPP_ERROR(node_->get_logger(),"Error in adding the object to the scene: '%s'", what.c_str());
    res->success = false;
    return;
  }
  RCLCPP_INFO(node_->get_logger(),"=================================================================");
  return;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  node_  = rclcpp::Node::make_shared("cnr_tf_named_object_loader",
                                     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  tf_named_objects_.reset(new cnr_tf_named_object_loader::TFNamedObjectsManager(node_));

  rclcpp::Service<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr service =
      node_->create_service<moveit_msgs::srv::ApplyPlanningScene>("load_tf_named_object", &add);
  RCLCPP_INFO(node_->get_logger(), "Ready to add two ints.");

  rclcpp::spin(node_);
  rclcpp::shutdown();

  return 0;
}
