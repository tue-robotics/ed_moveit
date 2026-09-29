#include "ed_moveit/moveit_plugin.h"

#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>

#include <geolib/Shape.h>
#include <geolib/Mesh.h>
#include <geolib/ros/msg_conversions.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/mesh.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>

#include <functional>


// ----------------------------------------------------------------------------------------------------

MoveitPlugin::MoveitPlugin() : world_model_(nullptr), update_req_(nullptr)
{
}

// ----------------------------------------------------------------------------------------------------

MoveitPlugin::~MoveitPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void MoveitPlugin::configure(tue::Configuration /*config*/)
{
}

// ----------------------------------------------------------------------------------------------------

void MoveitPlugin::initialize()
{
    // Own callback group + executor, spun in process(), so the service is only handled while the world model is valid
    cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    srv_publish_moveit_scene_ = node_->create_service<std_srvs::srv::Trigger>(
        "~/moveit_scene",
        std::bind(&MoveitPlugin::srvPublishMoveitScene, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), cb_group_);
    executor_.add_callback_group(cb_group_, node_->get_node_base_interface());

    moveit_scene_publisher_ = node_->create_publisher<moveit_msgs::msg::PlanningSceneWorld>("planning_scene_world", 1);
}

// ----------------------------------------------------------------------------------------------------

void MoveitPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    world_model_ = &world;
    update_req_ = &req;
    executor_.spin_some();
}

// ----------------------------------------------------------------------------------------------------

void MoveitPlugin::srvPublishMoveitScene(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    RCLCPP_INFO(node_->get_logger(), "[ED MOVEIT] Generating moveit planning scene");
    moveit_msgs::msg::PlanningSceneWorld msg;
    for (ed::WorldModel::const_iterator it = world_model_->begin(); it != world_model_->end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (!e->hasPose() || !e->collision() || e->existenceProbability() < 0.95 || e->hasFlag("self") || e->id() == "floor")
            continue;

        const geo::Mesh mesh = e->collision()->getMesh();

        shape_msgs::msg::Mesh mesh_msg;
        geo::convert(mesh, mesh_msg);

        moveit_msgs::msg::CollisionObject object_msg;
        object_msg.meshes.push_back(mesh_msg);

        //Pose is in 'map' frame. When publishing in own frame, pose can be zero.
        geometry_msgs::msg::Pose pose_msg;
        geo::convert(e->pose(), pose_msg);
        object_msg.mesh_poses.push_back(pose_msg);

        object_msg.operation = moveit_msgs::msg::CollisionObject::ADD;
        object_msg.id = e->id().str();
        object_msg.header.frame_id = "map";
        object_msg.header.stamp = node_->now();
        msg.collision_objects.push_back(object_msg);
    }

    moveit_scene_publisher_->publish(msg);
    res->success = true;
}

ED_REGISTER_PLUGIN(MoveitPlugin)
