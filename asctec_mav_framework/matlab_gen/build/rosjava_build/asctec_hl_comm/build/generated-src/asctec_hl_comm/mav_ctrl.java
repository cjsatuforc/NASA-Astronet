package asctec_hl_comm;

public interface mav_ctrl extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/mav_ctrl";
  static final java.lang.String _DEFINITION = "Header header\n\nint8        type        #message type\n\n# control commands, all units in SI units !!!\n# There are 3 operating modes of the helicopter:\n# 1. Acceleration: x, y, z correspond to x_dotdot etc... with the exception that yaw is angular velocity\n#    Inputs must be in body-coordinates\n#    Currently x~pitch, y~roll, z~thrust, units in rad and rad/s for yaw\n# 2. Velocity: x, y, z, yaw correspond to x_dot etc...\n#    Vehicle must have it\'s own pose estimation e.g. vision/GPS\n#    Inputs must be in body-coordinates\n#    Units in m/s and rad/s respectively\n# 3. Position\n#    Vehicle must have it\'s own pose estimation e.g. vision/GPS\n#    Inputs in fixed coordinate system\n#    v_max_* is only valid for this mode and limits the velocity a vehicle approaches the waypoint\n\nfloat32     x\nfloat32     y\nfloat32     z\nfloat32     yaw\nfloat32     v_max_xy\nfloat32     v_max_z\n\nint8 acceleration = 1\nint8 velocity = 2\nint8 position = 3\nint8 velocity_body = 4\nint8 position_body = 5\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  static final byte acceleration = 1;
  static final byte velocity = 2;
  static final byte position = 3;
  static final byte velocity_body = 4;
  static final byte position_body = 5;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  byte getType();
  void setType(byte value);
  float getX();
  void setX(float value);
  float getY();
  void setY(float value);
  float getZ();
  void setZ(float value);
  float getYaw();
  void setYaw(float value);
  float getVMaxXy();
  void setVMaxXy(float value);
  float getVMaxZ();
  void setVMaxZ(float value);
}
