package asctec_hl_comm;

public interface WaypointFeedback extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/WaypointFeedback";
  static final java.lang.String _DEFINITION = "#feedback\nHeader header\ngeometry_msgs/Point32   current_pos\nfloat32                 current_yaw\nstring                  status";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Point32 getCurrentPos();
  void setCurrentPos(geometry_msgs.Point32 value);
  float getCurrentYaw();
  void setCurrentYaw(float value);
  java.lang.String getStatus();
  void setStatus(java.lang.String value);
}
