package vicon_bridge;

public interface viconCalibrateSegment extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "vicon_bridge/viconCalibrateSegment";
  static final java.lang.String _DEFINITION = "string subject_name\nstring segment_name\nfloat64 z_offset\nint32 n_measurements\n---\nbool success\nstring status\ngeometry_msgs/PoseStamped pose";
}
