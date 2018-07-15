package vicon_bridge;

public interface Marker extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "vicon_bridge/Marker";
  static final java.lang.String _DEFINITION = "string marker_name\nstring subject_name\nstring segment_name\ngeometry_msgs/Point translation\nbool occluded\n";
  java.lang.String getMarkerName();
  void setMarkerName(java.lang.String value);
  java.lang.String getSubjectName();
  void setSubjectName(java.lang.String value);
  java.lang.String getSegmentName();
  void setSegmentName(java.lang.String value);
  geometry_msgs.Point getTranslation();
  void setTranslation(geometry_msgs.Point value);
  boolean getOccluded();
  void setOccluded(boolean value);
}
