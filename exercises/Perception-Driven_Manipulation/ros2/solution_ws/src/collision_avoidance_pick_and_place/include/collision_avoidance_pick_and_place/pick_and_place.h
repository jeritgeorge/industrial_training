#ifndef PICK_AND_PLACE_H_
#define PICK_AND_PLACE_H_

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/PlanningScene.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/msg/GetMotionPlan.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>

#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include <collision_avoidance_pick_and_place/pick_and_place_utilities.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <geometric_shapes/shape_operations.h>

// =============================== aliases ===============================
typedef actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction> GraspActionClient;
typedef boost::shared_ptr<GraspActionClient> GraspActionClientPtr;
typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;

namespace collision_avoidance_pick_and_place
{
	class PickAndPlace : public rclcpp::Node
	{
	public:

	// =============================== global members =====================================
		pick_and_place_config cfg;
		GraspActionClientPtr grasp_action_client_ptr;
		MoveGroupPtr move_group_ptr;
		TransformListenerPtr transform_listener_ptr;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
		rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher;
		rclcpp::Client<moveit_msgs::msg::GetMotionPlan>::SharedPtr motion_plan_client;
		rclcpp::Client<collision_avoidance_pick_and_place::msg::GetTargetPose>::SharedPtr target_recognition_client;

	// =============================== Task Functions ===============================
		void move_to_wait_position();

		void set_gripper(bool do_grasp);

		void set_attached_object(bool attach,
				const geometry_msgs::Pose &pose,moveit_msgs::RobotState &robot_state);

		void reset_world(bool refresh_octomap = true);

		geometry_msgs::Pose detect_box_pick();

		std::vector<geometry_msgs::Pose> create_pick_moves(geometry_msgs::Pose &box_pose);

		std::vector<geometry_msgs::Pose> create_place_moves();

		void pickup_box(std::vector<geometry_msgs::Pose>& pick_poses,const geometry_msgs::Pose& box_pose);

		void place_box(std::vector<geometry_msgs::Pose>& place_poses,const geometry_msgs::Pose& box_pose);


		bool create_motion_plan(const geometry_msgs::Pose &pose_target,
        const moveit_msgs::RobotState &start_robot_state,moveit::planning_interface::MoveGroupInterface::Plan &plan);

		void show_box(bool show=true)
		{
			// updating marker action
			cfg.MARKER_MESSAGE.action =
					show ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;

			// publish messages
			marker_publisher.publish(cfg.MARKER_MESSAGE);
		}

	protected:
	// =============================== private members =====================================


	};
}

#endif /* PICK_AND_PLACE_H_ */
