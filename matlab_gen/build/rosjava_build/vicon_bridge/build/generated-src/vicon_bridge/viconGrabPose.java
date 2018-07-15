package vicon_bridge;

public interface viconGrabPose extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "vicon_bridge/viconGrabPose";
  static final java.lang.String _DEFINITION = "string subject_name\nstring segment_name\nint32 n_measurements\n---\nbool success\ngeometry_msgs/PoseStamped pose";
}
