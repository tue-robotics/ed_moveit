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

void MoveitPlugin::configure(tue::Configuration config)
{

}

// ----------------------------------------------------------------------------------------------------

void MoveitPlugin::initialize()
{
    ros::NodeHandle nh("~");
    ros::NodeHandle nh_private;
    ros::AdvertiseServiceOptions opt_publish_moveit_scene_ =
            ros::AdvertiseServiceOptions::create<std_srvs::Trigger>(
                "moveit_scene", boost::bind(&MoveitPlugin::srvPublishMoveitScene, this, _1, _2), ros::VoidPtr(), &cb_queue_);
    srv_publish_moveit_scene_ = nh.advertiseService(opt_publish_moveit_scene_);

    moveit_scene_publisher_ = nh_private.advertise<moveit_msgs::PlanningSceneWorld>("planning_scene_world", 1);
}

// ----------------------------------------------------------------------------------------------------

void MoveitPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    world_model_ = &world;
    update_req_ = &req;
    cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

bool MoveitPlugin::srvPublishMoveitScene(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    ROS_INFO("[ED MOVEIT] Generating moveit planning scene");
    moveit_msgs::PlanningSceneWorld msg;
    for(ed::WorldModel::const_iterator it = world_model_->begin(); it != world_model_->end(); ++it)
        {
            const ed::EntityConstPtr& e = *it;

            if (!e->has_pose() || e->hasFlag("self") || e->id() == "floor")
                continue;

            shape_msgs::Mesh mesh_msg;
            if (e->shape())
            {
                const geo::Mesh& mesh = e->shape()->getMesh();
                geo::convert(mesh, mesh_msg);
            }
            else if (!e->convexHull().points.empty())
            {
                const ed::ConvexHull& convex_hull = e->convexHull();

                const geo::Pose3D& e_origin = e->pose();

                mesh_msg.vertices.resize(2 + 2*convex_hull.points.size());
                mesh_msg.triangles.resize(4*convex_hull.points.size());
                geo::convert((e_origin * geo::Pose3D(0, 0, convex_hull.z_min)).getOrigin(), mesh_msg.vertices[0]);
                geo::convert((e_origin * geo::Pose3D(0, 0, convex_hull.z_max)).getOrigin(), mesh_msg.vertices[1]);

                for(unsigned int i = 0; i < convex_hull.points.size(); ++i)
                {
                    unsigned int j = (i + 1) % convex_hull.points.size();

                    ROS_ERROR_STREAM("i: " << i << ", j: " << j << std::endl << ", begin: " << 4*i+2 << ", end: " << 4*i+5);

                    const geo::Vec2f& p1_2d = convex_hull.points[i];
//                    const geo::Vec2f& p2_2d = convex_hull.points[j];

                    geo::Pose3D p1_min = e_origin * geo::Pose3D(p1_2d.x, p1_2d.y, convex_hull.z_min);
                    geo::Pose3D p1_max = e_origin * geo::Pose3D(p1_2d.x, p1_2d.y, convex_hull.z_max);
//                    geo::Pose3D p2_min = e_origin * geo::Pose3D(p2_2d.x, p2_2d.y, convex_hull.z_min);
//                    geo::Pose3D p2_max = e_origin * geo::Pose3D(p2_2d.x, p2_2d.y, convex_hull.z_max);

                    geo::convert(p1_min.getOrigin(), mesh_msg.vertices[4*i+2]);
                    geo::convert(p1_max.getOrigin(), mesh_msg.vertices[4*i+3]);
//                    geo::convert(p2_min.getOrigin(), mesh_msg.vertices[4*j+2]);
//                    geo::convert(p2_max.getOrigin(), mesh_msg.vertices[4*j+3]);

                    mesh_msg.triangles[4*i].vertex_indices = {0, 4*i+3, 4*i+2}; // Bottom
                    mesh_msg.triangles[4*i+1].vertex_indices = {1, 4*j+2, 4*j+2}; // Top
                    mesh_msg.triangles[4*i+2].vertex_indices = {4*i+2, 4*i+3, 4*j+2}; // Side 1
                    mesh_msg.triangles[4*i+3].vertex_indices = {4*i+3, 4*j+3, 4*j+2}; // Side 2
                }
            }

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
