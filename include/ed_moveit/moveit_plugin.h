#ifndef ED_MOVEIT_PLUGIN_H_
#define ED_MOVEIT_PLUGIN_H_

#include <ed/plugin.h>

#include <ed/types.h>

// Communication
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>

// msgs&srvs
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>

class MoveitPlugin : public ed::Plugin
{

public:
    void initialize() override;

    void process(const ed::WorldModel& world, ed::UpdateRequest& req) override;

private:
    const ed::WorldModel* world_model_{nullptr};

    // Communication

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::executors::SingleThreadedExecutor executor_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_publish_moveit_scene_;
    void srvPublishMoveitScene(const std::shared_ptr<std_srvs::srv::Trigger::Request>& req,
                               const std::shared_ptr<std_srvs::srv::Trigger::Response>& res);
    rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr moveit_scene_publisher_;
};

#endif
