package asctec_hl_comm;

public interface Wgs84ToEnuResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/Wgs84ToEnuResponse";
  static final java.lang.String _DEFINITION = "float64 x\nfloat64 y\nfloat64 z";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  double getZ();
  void setZ(double value);
}
