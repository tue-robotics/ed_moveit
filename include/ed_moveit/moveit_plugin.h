#ifndef ED_MOVEIT_PLUGIN_H_
#define ED_MOVEIT_PLUGIN_H_

#include <ed/plugin.h>

#include <ed/types.h>

#include <rclcpp/rclcpp.hpp>

// Configuration
#include <tue/config/configuration.h>

//msgs&srvs
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <std_srvs/srv/trigger.hpp>

class MoveitPlugin : public ed::Plugin
{

public:

    MoveitPlugin();

    virtual ~MoveitPlugin();

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    const ed::WorldModel* world_model_;

    ed::UpdateRequest* update_req_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_publish_moveit_scene_;
    void srvPublishMoveitScene(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr moveit_scene_publisher_;

};

#endif
