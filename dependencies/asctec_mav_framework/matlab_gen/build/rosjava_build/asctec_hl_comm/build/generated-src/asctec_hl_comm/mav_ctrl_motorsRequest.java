package asctec_hl_comm;

public interface mav_ctrl_motorsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/mav_ctrl_motorsRequest";
  static final java.lang.String _DEFINITION = "bool      startMotors         # starts/stops the motors\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  boolean getStartMotors();
  void setStartMotors(boolean value);
}
