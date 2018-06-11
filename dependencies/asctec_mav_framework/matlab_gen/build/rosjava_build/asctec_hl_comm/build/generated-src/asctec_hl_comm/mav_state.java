package asctec_hl_comm;

public interface mav_state extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/mav_state";
  static final java.lang.String _DEFINITION = "Header header\ngeometry_msgs/Pose          pose\ngeometry_msgs/Vector3       velocity";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Pose getPose();
  void setPose(geometry_msgs.Pose value);
  geometry_msgs.Vector3 getVelocity();
  void setVelocity(geometry_msgs.Vector3 value);
}
