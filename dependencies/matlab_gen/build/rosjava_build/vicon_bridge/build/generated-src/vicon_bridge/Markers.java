package vicon_bridge;

public interface Markers extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "vicon_bridge/Markers";
  static final java.lang.String _DEFINITION = "Header header\nuint32 frame_number # Vicon Frame Number\nvicon_bridge/Marker[] markers # Array of Markers";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  int getFrameNumber();
  void setFrameNumber(int value);
  java.util.List<vicon_bridge.Marker> getMarkers();
  void setMarkers(java.util.List<vicon_bridge.Marker> value);
}
