package asctec_hl_comm;

public interface mav_rcdata extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/mav_rcdata";
  static final java.lang.String _DEFINITION = "Header header\n\nuint16[8] channel";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  short[] getChannel();
  void setChannel(short[] value);
}
