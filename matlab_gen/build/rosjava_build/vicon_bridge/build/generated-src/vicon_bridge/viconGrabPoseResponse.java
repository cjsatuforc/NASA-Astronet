package vicon_bridge;

public interface viconGrabPoseResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "vicon_bridge/viconGrabPoseResponse";
  static final java.lang.String _DEFINITION = "bool success\ngeometry_msgs/PoseStamped pose";
  boolean getSuccess();
  void setSuccess(boolean value);
  geometry_msgs.PoseStamped getPose();
  void setPose(geometry_msgs.PoseStamped value);
}
