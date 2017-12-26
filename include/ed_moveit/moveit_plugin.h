#ifndef ED_MOVEIT_PLUGIN_H_
#define ED_MOVEIT_PLUGIN_H_

#include <ed/plugin.h>

#include <ed/types.h>

// Communication
#include <ros/callback_queue.h>
#include <ros/service_server.h>
#include <ros/publisher.h>

// Configuration
#include <tue/config/configuration.h>

//msgs&srvs
#include <moveit_msgs/PlanningSceneWorld.h>
#include <std_srvs/Trigger.h>

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

    // Communication

    ros::CallbackQueue cb_queue_;


    ros::ServiceServer srv_publish_moveit_scene_;
    bool srvPublishMoveitScene(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    ros::Publisher moveit_scene_publisher_;

};

#endif
