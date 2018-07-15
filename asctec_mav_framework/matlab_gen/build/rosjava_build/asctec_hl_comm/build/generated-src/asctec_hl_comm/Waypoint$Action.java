package asctec_hl_comm;

public interface Waypoint$Action extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "asctec_hl_comm/Waypoint$Action";
  static final java.lang.String _DEFINITION = "#goal\nHeader header\ngeometry_msgs/Point32   goal_pos\nfloat32                 goal_yaw\ngeometry_msgs/Point32   max_speed\nfloat32 \t\t\t\taccuracy_position        # determines the radius around the goal within the goal is considered as reached\nfloat32                 accuracy_orientation     # determines the range within the goal orientation is considered as reached, for the heli only yaw\nfloat32                 timeout                  # timeout in [s] after which flying to the waypoint stops\n---\n#result\nHeader header\ngeometry_msgs/Point32   result_pos\nfloat32                 result_yaw\nstring                  status\n---\n#feedback\nHeader header\ngeometry_msgs/Point32   current_pos\nfloat32                 current_yaw\nstring                  status\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
}
