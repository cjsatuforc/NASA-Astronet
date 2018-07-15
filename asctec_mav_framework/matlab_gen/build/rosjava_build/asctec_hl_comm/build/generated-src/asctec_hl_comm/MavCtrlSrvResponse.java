package asctec_hl_comm;

public interface MavCtrlSrvResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/MavCtrlSrvResponse";
  static final java.lang.String _DEFINITION = "mav_ctrl ctrl_result";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  asctec_hl_comm.mav_ctrl getCtrlResult();
  void setCtrlResult(asctec_hl_comm.mav_ctrl value);
}
