package asctec_hl_comm;

public interface mav_ctrl_motorsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/mav_ctrl_motorsResponse";
  static final java.lang.String _DEFINITION = "bool      motorsRunning       # are the motors running? ";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  boolean getMotorsRunning();
  void setMotorsRunning(boolean value);
}
