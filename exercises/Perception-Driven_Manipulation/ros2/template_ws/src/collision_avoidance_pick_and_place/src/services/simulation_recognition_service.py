#!/usr/bin/env python
import roslib; roslib.load_manifest('collision_avoidance_pick_and_place')
import rclpy
import tf
import shape_msgs
import collision_avoidance_pick_and_place.srv
from collision_avoidance_pick_and_place.srv import *
from rclpy import Node


#constants
TARGET_RECOGNITION_SERVICE = "target_recognition";

#variables
tf_listener = None

class SimulationRecognitionService(Node):
	def __init__(self):
		super().__init__('simulation_recognition_node')
		self.srv = self.create_service(GetTargetPose, TARGET_RECOGNITION_SERVICE, self.recognition_callback)

#server callback
	def recognition_callback(req):


		ar_frame = req.ar_tag_frame_id
		world_frame = req.world_frame_id
		shape = req.shape


		# response object
		res = GetTargetPoseResponse()

		# lookup tranform
		try:
			(trans,rot) = tf_listener.lookupTransform(world_frame,ar_frame,rospy.Time(0))
			rospy.loginfo("ar_frame '%s' relative to '%s' detected",ar_frame,world_frame)

			#modifying height
			height = shape.dimensions[2] if (len(shape.dimensions)==3) else trans[2]

			# creating pose
			pose = geometry_msgs.msg.Pose()
			pose.position.x = trans[0]
			pose.position.y = trans[1]
			pose.position.z = height
			pose.orientation.x = rot[1]
			pose.orientation.y = rot[2]
			pose.orientation.z = rot[3]
			pose.orientation.w = rot[0]


			res.target_pose = pose
			res.succeeded = True


		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logerr("ar_frame '%s' relative to '%s' not found",ar_frame,world_frame)
			res.succeeded = False


		return res

if __name__ == "__main__":

	rclpy.init(args=args)
	# creating listener
	tf_listener = tf.TransformListener()

	# creating server
	server = SimulationRecognitionService()
	rclpy.loginfo("recognition service server ready")
	rclpy.spin(server)
    rclpy.shutdown()






