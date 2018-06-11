package asctec_hl_comm;

public interface WaypointGoal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/WaypointGoal";
  static final java.lang.String _DEFINITION = "#goal\nHeader header\ngeometry_msgs/Point32   goal_pos\nfloat32                 goal_yaw\ngeometry_msgs/Point32   max_speed\nfloat32 \t\t\t\taccuracy_position        # determines the radius around the goal within the goal is considered as reached\nfloat32                 accuracy_orientation     # determines the range within the goal orientation is considered as reached, for the heli only yaw\nfloat32                 timeout                  # timeout in [s] after which flying to the waypoint stops\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Point32 getGoalPos();
  void setGoalPos(geometry_msgs.Point32 value);
  float getGoalYaw();
  void setGoalYaw(float value);
  geometry_msgs.Point32 getMaxSpeed();
  void setMaxSpeed(geometry_msgs.Point32 value);
  float getAccuracyPosition();
  void setAccuracyPosition(float value);
  float getAccuracyOrientation();
  void setAccuracyOrientation(float value);
  float getTimeout();
  void setTimeout(float value);
}
