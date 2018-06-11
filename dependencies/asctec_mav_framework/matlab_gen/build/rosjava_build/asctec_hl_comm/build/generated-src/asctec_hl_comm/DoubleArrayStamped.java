package asctec_hl_comm;

public interface DoubleArrayStamped extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/DoubleArrayStamped";
  static final java.lang.String _DEFINITION = "Header    header\nfloat64[] data";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  double[] getData();
  void setData(double[] value);
}
