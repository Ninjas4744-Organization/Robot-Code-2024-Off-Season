package frc.robot.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  //  private static Vision _instance;
  //
  //  public static Vision getInstance() {
  //    return null;
  //    if (_instance == null) _instance = new Vision();
  //    return _instance;
  //  }
  //
  //  private VisionCamera[] _cameras;
  //  private VisionEstimation[] _estimationsData;
  //  private HashMap<String, VisionOutput> _outputs;
  //
  //  private Vision() {
  //    HashMap<String, Transform3d> camerasPoses = Constants.VisionConstants.getCamerasPoses();
  //    String[] camerasNames = camerasPoses.keySet().toArray(new String[0]);
  //
  //    _cameras = new VisionCamera[camerasNames.length];
  //
  //    for (int i = 0; i < camerasPoses.size(); i++) {
  //      _cameras[i] = new VisionCamera(camerasNames[i], camerasPoses.get(camerasNames[i]));
  //    }
  //
  //    _estimationsData = new VisionEstimation[_cameras.length];
  //    _outputs = new HashMap<String, VisionOutput>();
  //    for (int i = 0; i < _cameras.length; i++) {
  //      VisionOutput output = new VisionOutput();
  //      _outputs.put(_cameras[i].getName(), output);
  //    }
  //  }
  //
  //  public void periodic() {
  //    for (int i = 0; i < _cameras.length; i++) {
  //      VisionOutput output = _cameras[i].Update();
  //      _outputs.put(_cameras[i].getName(), output);
  //      _estimationsData[i] =
  //          new VisionEstimation(output.robotPose, output.timestamp, output.hasTargets);
  //    }
  //  }
  //
  //  /**
  //   * @return an array of each camera's robot pose, the time when this pose was detected and if
  // it
  //   *     has targets
  //   */
  //  public VisionEstimation[] getVisionEstimations() {
  //    return _estimationsData;
  //  }
  //
  //  /**
  //   * @param camera - the name of the name of the camera to get info from
  //   * @return distance from the closest tag to this camera
  //   */
  //  public double getClosestTagDistance(String camera) {
  //    return _outputs.get(camera).closestTagDist;
  //  }
  //
  //  /**
  //   * @param camera - the name of the camera to get info from
  //   * @return closest tag to this camera
  //   */
  //  public AprilTag getClosestTag(String camera) {
  //    return _outputs.get(camera).closestTag;
  //  }
  //
  //  /**
  //   * @param camera - the name of the camera to get info from
  //   * @return distance from the farthest tag to this camera
  //   */
  //  public double getFarthestTagDistance(String camera) {
  //    return _outputs.get(camera).farthestTagDist;
  //  }
  //
  //  /**
  //   * @param camera - the name of the camera to get info from
  //   * @return farthest tag to this camera
  //   */
  //  public AprilTag getFarthestTag(String camera) {
  //    return _outputs.get(camera).farthestTag;
  //  }
  //
  //  /**
  //   * @param camera - the name of the camera to get info from
  //   * @return ambiguity of the most ambiguous tag from this camera
  //   */
  //  public double getMostAmbiguousTagAmbiguity(String camera) {
  //    return _outputs.get(camera).maxAmbiguity;
  //  }
  //
  //  /**
  //   * @param camera - the name of the camera to get info from
  //   * @return most ambiguous tag from this camera
  //   */
  //  public AprilTag getMostAmbiguousTag(String camera) {
  //    return _outputs.get(camera).maxAmbiguityTag;
  //  }
  //
  //  /**
  //   * @param camera - the name of the camera to get info from
  //   * @return if this camera has targets
  //   */
  //  public boolean hasTargets(String camera) {
  //    return _outputs.get(camera).hasTargets;
  //  }
  //
  //  /**
  //   * @return if any of the cameras have targets
  //   */
  //  public boolean hasTargets() {
  //    boolean hasTargets = false;
  //
  //    for (String camera : _outputs.keySet()) {
  //      if (_outputs.get(camera).hasTargets) {
  //        hasTargets = true;
  //        break;
  //      }
  //    }
  //
  //    return hasTargets;
  //  }
  //
  //  public boolean atAmp() {
  //    return RobotState.getRobotPose()
  //            .getTranslation()
  //            .getDistance(VisionConstants.getAmpPose().getTranslation())
  //        < 0.5;
  //  }
  //
  //  public boolean atSource() {
  //    return RobotState.getRobotPose()
  //            .getTranslation()
  //            .getDistance(VisionConstants.getSourcePose().getTranslation())
  //        < 0.5;
  //  }
  //
  //  public boolean atSpeaker() {
  //    return RobotState.getRobotPose()
  //            .getTranslation()
  //            .getDistance(VisionConstants.getSpeakerPose().getTranslation())
  //        < 2.5;
  //  }
}
