package asctec_hl_comm;

public interface Wgs84ToEnuRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/Wgs84ToEnuRequest";
  static final java.lang.String _DEFINITION = "float64 lat\nfloat64 lon\nfloat64 alt\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  double getLat();
  void setLat(double value);
  double getLon();
  void setLon(double value);
  double getAlt();
  void setAlt(double value);
}
