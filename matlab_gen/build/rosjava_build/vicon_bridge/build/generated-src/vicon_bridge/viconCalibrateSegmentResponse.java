package vicon_bridge;

public interface viconCalibrateSegmentResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "vicon_bridge/viconCalibrateSegmentResponse";
  static final java.lang.String _DEFINITION = "bool success\nstring status\ngeometry_msgs/PoseStamped pose";
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatus();
  void setStatus(java.lang.String value);
  geometry_msgs.PoseStamped getPose();
  void setPose(geometry_msgs.PoseStamped value);
}
