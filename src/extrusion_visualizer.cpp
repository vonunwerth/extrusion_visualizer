#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <tf/tf.h>
#include <rosbag/bag.h>
#include <cmath>
#include <csignal>
#include <rosbag/view.h>
#include <cstdio>

/**
 * Calculation result data structure
 */
visualization_msgs::Marker elements;

//TODO ggf größe von dauer abhängig machen also wirklich "Druck" simulieren über Größe der Elemente
/**
 * Visualizes extrusion elements
 * @param argc Not used
 * @param argv Not used
 * @return 0 on success
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "extrusion_visualizer");
    ros::NodeHandle node_handle("~"); // Allow access to private ROS parameters

    // Load the planning group parameter from the configuration
    std::string planning_group;
    if (node_handle.getParam("planning_group", planning_group)) ROS_INFO_STREAM("Planning group: " << planning_group);
    else {
        planning_group = "manipulator"; // Default value
        ROS_INFO_STREAM(
                "Param \"planning group\" not provided. simple_reachability will use default planning group \"manipulator\".");
    }

    // Load the planning group parameter from the configuration
    std::string base_frame;
    if (node_handle.getParam("base_frame", base_frame)) ROS_INFO_STREAM("Base frame: " << planning_group);
    else {
        base_frame = "odom"; // Default value
        ROS_INFO_STREAM(
                "Param \"base_frame\" not provided. extrusion_visualizer will use default frame \"odom\".");
    }

    // Load the base_link parameter from the configuration
    std::string base_link;
    node_handle.param<std::string>("manipulator_base_link", base_link, "base_link");

    // Load the resolution parameter from the configuration
    double resolution; // Resolution in meter
    node_handle.param<double>("resolution", resolution, 0.05);
    ROS_INFO_STREAM("Extrusion Element Resolution: " << resolution);

    // Loads the extrusion diameter parameter from the configuration file
    double extrusion_diameter;
    node_handle.param("extrusion_diameter", extrusion_diameter, 0.1);

    // Which type of extrusion elements should be placed
    int type;
    node_handle.param("type", type, 0);

    float rate;
    node_handle.param("rate", rate, 1.0f);
    ros::Rate loop_rate(rate);

    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const robot_model::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    moveit::core::RobotState robot_state(robot_model_loader.getModel());
    kinematic_state->setToDefaultValues();
    //Set reference frame for planning to the ur base link
    move_group.setPoseReferenceFrame(
            base_link);

    std_msgs::ColorRGBA color;
    color.r = 200;
    color.g = 200;
    color.b = 200;
    color.a = 1.0f;

    geometry_msgs::Quaternion element_rot;
    element_rot.w = 1;

    elements.header.frame_id = base_frame;
    elements.header.stamp = ros::Time(0);
    elements.ns = "points";
    elements.action = visualization_msgs::Marker::ADD;
    elements.id = 0;
    elements.type = visualization_msgs::Marker::CUBE_LIST;
    elements.scale.x = extrusion_diameter; //TODO depending on resolution
    elements.scale.y = extrusion_diameter;
    elements.scale.z = extrusion_diameter;
    elements.color = color;
    elements.pose.orientation = element_rot;

    ros::Publisher extrusion_pub = node_handle.advertise<visualization_msgs::Marker>("extruded_material", 10);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    while (ros::ok()) {
        //TODO Subscribe to coord of the ee, check abweichung zu letztem schleifendurchgang und male evtl neues element
        geometry_msgs::Pose p2 = move_group.getCurrentPose().pose;
        geometry_msgs::Pose p = move_group.getCurrentPose("ur10_base_link").pose;
        elements.points.push_back(p2.position);
        extrusion_pub.publish(elements);
        //TODO checken wie viel mist schon in elements drin ist
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}