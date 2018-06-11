package asctec_hl_comm;

public interface mav_ekf extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/mav_ekf";
  static final java.lang.String _DEFINITION = "Header header\ngeometry_msgs/Vector3       angular_velocity\ngeometry_msgs/Vector3       linear_acceleration\nfloat32[]                   state\nint32                       flag\n\nuint32                      ignore_state = 0\nuint32                      current_state = 1\nuint32                      initialization = 2\nuint32                      state_correction = 3\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  static final int ignore_state = 0;
  static final int current_state = 1;
  static final int initialization = 2;
  static final int state_correction = 3;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Vector3 getAngularVelocity();
  void setAngularVelocity(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getLinearAcceleration();
  void setLinearAcceleration(geometry_msgs.Vector3 value);
  float[] getState();
  void setState(float[] value);
  int getFlag();
  void setFlag(int value);
}
