package asctec_hl_comm;

public interface PositionWithCovarianceStamped extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/PositionWithCovarianceStamped";
  static final java.lang.String _DEFINITION = "Header header\ngeometry_msgs/Point position\nfloat64[9] covariance";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Point getPosition();
  void setPosition(geometry_msgs.Point value);
  double[] getCovariance();
  void setCovariance(double[] value);
}
