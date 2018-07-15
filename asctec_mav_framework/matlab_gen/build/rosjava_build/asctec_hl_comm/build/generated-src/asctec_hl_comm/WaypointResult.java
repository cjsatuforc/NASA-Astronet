package asctec_hl_comm;

public interface WaypointResult extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/WaypointResult";
  static final java.lang.String _DEFINITION = "#result\nHeader header\ngeometry_msgs/Point32   result_pos\nfloat32                 result_yaw\nstring                  status\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Point32 getResultPos();
  void setResultPos(geometry_msgs.Point32 value);
  float getResultYaw();
  void setResultYaw(float value);
  java.lang.String getStatus();
  void setStatus(java.lang.String value);
}
