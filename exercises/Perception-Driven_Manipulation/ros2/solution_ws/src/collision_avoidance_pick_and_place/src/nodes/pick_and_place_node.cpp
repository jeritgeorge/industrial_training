#include <collision_avoidance_pick_and_place/pick_and_place.h>

using namespace collision_avoidance_pick_and_place;

class PickAndPlace : public rclcpp::Node
{
    public:
        PickAndPlace() : Node("pick_and_place_node")
        {
            // reading parameters
            if(application.cfg.init())
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "Parameters successfully read");
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Parameters not found");
                return 0;
            }

            // marker publisher
            application.marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>(
                    application.cfg.MARKER_TOPIC,1);

            // planning scene publisher
            application.planning_scene_publisher = this->create_publisher<moveit_msgs::msg::PlanningScene>(
                    application.cfg.PLANNING_SCENE_TOPIC,1);

            // moveit interface
            application.move_group_ptr = MoveGroupPtr(
                new moveit::planning_interface::MoveGroupInterface(application.cfg.ARM_GROUP_NAME));
            application.move_group_ptr->setPlannerId("RRTConnectkConfigDefault");

            // motion plan client
            application.motion_plan_client = this->create_client<moveit_msgs::msg::GetMotionPlan>
                (application.cfg.MOTION_PLAN_SERVICE);

            // transform listener
            application.transform_listener_ptr = TransformListenerPtr(new tf::TransformListener());

            // marker publisher (rviz visualization)
            application.marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>(
                    application.cfg.MARKER_TOPIC,1);

            // target recognition client (perception)
            application.target_recognition_client = this->create_client<collision_avoidance_pick_and_place::msg::GetTargetPose>(
                    application.cfg.TARGET_RECOGNITION_SERVICE);

            // grasp action client (vacuum gripper)
            application.grasp_action_client_ptr = this->create_client<GraspActionClientPtr>(
                    this, application.cfg.GRASP_ACTION_NAME, true));

        }
}

// =============================== Main Thread ===============================
int main(int argc,char** argv)
{
  geometry_msgs::msg::Pose box_pose;
  std::vector<geometry_msgs::msg::Pose> pick_poses, place_poses;

  /* =========================================================================================*/
  /*	INITIALIZING ROS NODE
      Goal:
      - Observe all steps needed to properly initialize a ros node.
      - Look into the 'cfg' member of PickAndPlace to take notice of the parameters that
        are available for the rest of the program. */
  /* =========================================================================================*/

  // ros initialization
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PickAndPlace>());

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // creating pick and place application instance
  PickAndPlace application;

  // waiting to establish connections
  while(ros::ok() &&
      !application.grasp_action_client_ptr->waitForServer(ros::Duration(2.0f)))
  {
    ROS_INFO_STREAM("Waiting for grasp action servers");
  }

  if(ros::ok() && !application.target_recognition_client.waitForExistence(ros::Duration(2.0f)))
  {
	  ROS_INFO_STREAM("Waiting for service'"<<application.cfg.TARGET_RECOGNITION_SERVICE<<"'");
  }


  /* ========================================*/
  /* Pick & Place Tasks                      */
  /* ========================================*/

  // move to a "clear" position
  application.move_to_wait_position();

  // turn off vacuum gripper
  application.set_gripper(false);

  // get the box position and orientation
  box_pose = application.detect_box_pick();

  // build a sequence of poses to "pick" the box
  pick_poses = application.create_pick_moves(box_pose);

  // plan/execute the sequence of "pick" moves
  application.pickup_box(pick_poses,box_pose);

  // build a sequence of poses to "place" the box
  place_poses = application.create_place_moves();

  // plan/execute the "place" moves
  application.place_box(place_poses,box_pose);

  // move back to the "clear" position
  application.move_to_wait_position();

  return 0;
}
