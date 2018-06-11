package asctec_hl_comm;

public interface MotorSpeed extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/MotorSpeed";
  static final java.lang.String _DEFINITION = "Header header\nint16[6] motor_speed # motor speeds, rxplot doesn\'t like uint8 :( ...";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  short[] getMotorSpeed();
  void setMotorSpeed(short[] value);
}
