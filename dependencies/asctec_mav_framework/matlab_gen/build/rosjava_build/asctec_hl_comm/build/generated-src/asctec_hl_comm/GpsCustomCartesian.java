package asctec_hl_comm;

public interface GpsCustomCartesian extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/GpsCustomCartesian";
  static final java.lang.String _DEFINITION = "\nHeader header\n\ngeometry_msgs/Point   position\nfloat64[9]            position_covariance\nfloat64               velocity_x\nfloat64               velocity_y\nfloat64[4]            velocity_covariance\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Point getPosition();
  void setPosition(geometry_msgs.Point value);
  double[] getPositionCovariance();
  void setPositionCovariance(double[] value);
  double getVelocityX();
  void setVelocityX(double value);
  double getVelocityY();
  void setVelocityY(double value);
  double[] getVelocityCovariance();
  void setVelocityCovariance(double[] value);
}
