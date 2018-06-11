package asctec_hl_comm;

public interface GpsCustom extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/GpsCustom";
  static final java.lang.String _DEFINITION = "# Navigation Satellite fix for any Global Navigation Satellite System\n#\n# Specified using the WGS 84 reference ellipsoid\n\n# Header specifies ROS time and frame of reference for this fix.\nHeader header\n\n# satellite fix status information\nsensor_msgs/NavSatStatus status\n\n# Latitude [degrees]. Positive is north of equator; negative is south.\nfloat64 latitude\n\n# Longitude [degrees]. Positive is east of prime meridian; negative is west.\nfloat64 longitude\n\n# Altitude [m]. Positive is above the WGS 84 ellipsoid.\nfloat64 altitude\n\n# Position covariance [m^2] defined relative to a tangential plane\n# through the reported position. The components are East, North, and\n# Up (ENU), in row-major order.\n#\n# Beware: this coordinate system exhibits singularities at the poles.\n\nfloat64[9] position_covariance\n\nuint8 position_covariance_type\n\nfloat64 pressure_height\n\nfloat64 velocity_x\nfloat64 velocity_y\nfloat64[4] velocity_covariance";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  sensor_msgs.NavSatStatus getStatus();
  void setStatus(sensor_msgs.NavSatStatus value);
  double getLatitude();
  void setLatitude(double value);
  double getLongitude();
  void setLongitude(double value);
  double getAltitude();
  void setAltitude(double value);
  double[] getPositionCovariance();
  void setPositionCovariance(double[] value);
  byte getPositionCovarianceType();
  void setPositionCovarianceType(byte value);
  double getPressureHeight();
  void setPressureHeight(double value);
  double getVelocityX();
  void setVelocityX(double value);
  double getVelocityY();
  void setVelocityY(double value);
  double[] getVelocityCovariance();
  void setVelocityCovariance(double[] value);
}
