package asctec_hl_comm;

public interface mav_imu extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/mav_imu";
  static final java.lang.String _DEFINITION = "Header header\ngeometry_msgs/Quaternion    orientation\ngeometry_msgs/Vector3       angular_velocity\ngeometry_msgs/Vector3       acceleration\nfloat64                     height\nfloat64                     differential_height";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  geometry_msgs.Vector3 getAngularVelocity();
  void setAngularVelocity(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getAcceleration();
  void setAcceleration(geometry_msgs.Vector3 value);
  double getHeight();
  void setHeight(double value);
  double getDifferentialHeight();
  void setDifferentialHeight(double value);
}
