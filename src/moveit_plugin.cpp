#include "ed_moveit/moveit_plugin.h"

#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>

#include <ros/node_handle.h>
#include <ros/advertise_service_options.h>

#include <geolib/Shape.h>
#include <geolib/Mesh.h>
#include <geolib/ros/msg_conversions.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/Mesh.h>


// ----------------------------------------------------------------------------------------------------

MoveitPlugin::MoveitPlugin()
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
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::AdvertiseServiceOptions opt_publish_moveit_scene_ =
            ros::AdvertiseServiceOptions::create<std_srvs::Trigger>(
                "moveit_scene", boost::bind(&MoveitPlugin::srvPublishMoveitScene, this, _1, _2), ros::VoidPtr(), &cb_queue_);
    srv_publish_moveit_scene_ = nh_private.advertiseService(opt_publish_moveit_scene_);

    moveit_scene_publisher_ = nh.advertise<moveit_msgs::PlanningSceneWorld>("planning_scene_world", 1);
}

// ----------------------------------------------------------------------------------------------------

void MoveitPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    world_model_ = &world;
    update_req_ = &req;
    cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

bool MoveitPlugin::srvPublishMoveitScene(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res)
{
    ROS_INFO("[ED MOVEIT] Generating moveit planning scene");
    moveit_msgs::PlanningSceneWorld msg;
    for(ed::WorldModel::const_iterator it = world_model_->begin(); it != world_model_->end(); ++it)
        {
            const ed::EntityConstPtr& e = *it;

            if (!e->has_pose() || !e->shape() || e->existenceProbability() < 0.95 || e->hasFlag("self") || e->id() == "floor")
                continue;

            const geo::Mesh mesh = e->shape()->getMesh();

            shape_msgs::Mesh mesh_msg;
            geo::convert(mesh, mesh_msg);

            moveit_msgs::CollisionObject object_msg;
            object_msg.meshes.push_back(mesh_msg);

            //Pose is in 'map' frame. When publishing in own frame, pose can be zero.
            geometry_msgs::Pose pose_msg;
            geo::convert(e->pose(), pose_msg);
            object_msg.mesh_poses.push_back(pose_msg);

            object_msg.operation = moveit_msgs::CollisionObject::ADD;
            object_msg.id = e->id().str();
            object_msg.header.frame_id = "map";
            object_msg.header.stamp = ros::Time::now();
            msg.collision_objects.push_back(object_msg);
        }

    moveit_scene_publisher_.publish(msg);
    res.success = true;
    return true;
}

ED_REGISTER_PLUGIN(MoveitPlugin)
