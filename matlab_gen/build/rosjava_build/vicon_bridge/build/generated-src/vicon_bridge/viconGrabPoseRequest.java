package vicon_bridge;

public interface viconGrabPoseRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "vicon_bridge/viconGrabPoseRequest";
  static final java.lang.String _DEFINITION = "string subject_name\nstring segment_name\nint32 n_measurements\n";
  java.lang.String getSubjectName();
  void setSubjectName(java.lang.String value);
  java.lang.String getSegmentName();
  void setSegmentName(java.lang.String value);
  int getNMeasurements();
  void setNMeasurements(int value);
}
