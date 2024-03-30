package frc.robot.utilities;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/** Reports our expected, desired, and actual poses to dashboards */
public class PoseEstimator extends SubsystemBase {
  private static PoseEstimator instance;

  public static PoseEstimator getInstance() {
    if (instance == null) instance = new PoseEstimator();
    return instance;
  }

  // Constants.PoseConstants config;
  private Pose2d odometryPose = new Pose2d();
  private Pose2d estimatePose = new Pose2d();

  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveSubsystem swerveSubsystem;

  private ShuffleboardTab tab = Shuffleboard.getTab("Odometry Data");
  private GenericEntry xPoseDiffEntry = tab.add("XOdom Diff", 0).getEntry();
  private GenericEntry yPoseDiffEntry = tab.add("YODom Diff", 0).getEntry();
  private GenericEntry totalDiffEntry = tab.add("totalDiff", 0).getEntry();
  private GenericEntry rToSpeaker = tab.add("Distance to Speaker", 0).getEntry();
  private GenericEntry aprilTagTelemEntry = tab.add("Has AprilTag Telemetry", false).getEntry();

  private PoseEstimator() {
    // config = new Constants.PoseConstants();
    swerveSubsystem = new SwerveSubsystem();

    // Maxswerve Version from MAXSwerve.java in core
    poseEstimator = new SwerveDrivePoseEstimator(
      Constants.SwerveConstants.SwerveKinematics,
      swerveSubsystem.getYawRotation2d(),
      swerveSubsystem.getSwerveModulePositions(),
      swerveSubsystem.getPose(),
      createStateStdDevs(
        Constants.PoseConstants.kPositionStdDevX,
        Constants.PoseConstants.kPositionStdDevY,
        Constants.PoseConstants.kPositionStdDevTheta),
      createVisionMeasurementStdDevs(
        Constants.PoseConstants.kVisionStdDevX,
        Constants.PoseConstants.kVisionStdDevY,
        Constants.PoseConstants.kVisionStdDevTheta)
    );
  }

  @Override
  public void periodic() {
    updateOdometryEstimate(); // Updates using wheel encoder data only
    // Updates using the vision estimate
    Pose2d tempEstimatePose = VisionSubsystem.getInstance().visionBotPose();
    if (Constants.VisionConstants.IS_LIMELIGHT_MODE && tempEstimatePose != null
        && (tempEstimatePose.getX() > Constants.VisionConstants.VISION_X_MAX_CUTOFF || tempEstimatePose.getX() < Constants.VisionConstants.VISION_X_MIN_CUTOFF)) { // Limelight mode
      if (isEstimateReady(tempEstimatePose)) { // Does making so many bot pose variables impact accuracy?
        double currentTimestamp = VisionSubsystem.getInstance().getTimestampSeconds(VisionSubsystem.getInstance().getTotalLatency());
        addVisionMeasurement(tempEstimatePose, currentTimestamp);
      }
    }
    // TODO Photonvision mode - Needs editing and filtering
    if (Constants.VisionConstants.IS_PHOTON_VISION_MODE && tempEstimatePose != null
        && (tempEstimatePose.getX() > Constants.VisionConstants.VISION_X_MAX_CUTOFF || tempEstimatePose.getX() < Constants.VisionConstants.VISION_X_MIN_CUTOFF)) { // Limelight mode
      if (isEstimateReady(tempEstimatePose)) { // Does making so many bot pose variables impact accuracy?
        double photonTimestamp = VisionSubsystem.getInstance().getPhotonTimestamp();
        addVisionMeasurement(tempEstimatePose, photonTimestamp);
        aprilTagTelemEntry.setBoolean(true);
      }
      else{
        aprilTagTelemEntry.setBoolean(false);
      }
    }

    //UNTESTED - ALWAYS SETS DRIVETRAIN ODOMETRY TO THE POSE-ESTIMATOR ODOMETRY
    //NOT GREAT FOR ERROR CHECKING POSE ESTIMATOR! - SET TO FALSE
    if (Constants.VisionConstants.VISION_OVERRIDE_ENABLED) {
      swerveSubsystem.resetSwerveOdometry(getPosition());
    }

    // Update for telemetry
    setEstimatedPose(getPosition());
    setOdometryPose(swerveSubsystem.getPose());

    double xDiff = estimatePose.getX() - odometryPose.getX();
    double yDiff = estimatePose.getY() - odometryPose.getY();

    xPoseDiffEntry.setDouble(xDiff);
    yPoseDiffEntry.setDouble(yDiff);
    totalDiffEntry.setDouble(Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2)));

    double xAvg = (estimatePose.getX() + odometryPose.getX()) / 2;
    double yAvg = (estimatePose.getY() + odometryPose.getY()) / 2;
    swerveSubsystem.resetSwerveOdometry(new Pose2d(xAvg, yAvg, swerveSubsystem.getYawRotation2d()));

    Translation2d currentTranslation = getPosition().getTranslation();
    Pose2d targetCoordinate = Constants.DriverConstants.IS_ALLIANCE_BLUE ? FieldConstants.Coordinates.BLUE_SPEAKER : FieldConstants.Coordinates.RED_SPEAKER;;

    double targetVectorLength = currentTranslation.getDistance(targetCoordinate.getTranslation());
    rToSpeaker.setDouble(targetVectorLength);

  }
  
  public Double getDistanceToPose(Translation2d pose) {
        return getPosition().getTranslation().getDistance(pose);
  }

  /**
   * Helper method for comparing vision pose against odometry pose. Does not account for difference
   * in rotation. Will return false vision if it sees no targets or if the vision estimated pose is
   * too far from the odometry estimate
   *
   * @return whether or not pose should be added to estimate or not
   */
  public boolean isEstimateReady(Pose2d pose) {
    /* Disregard Vision if there are no targets in view */
    if (!VisionSubsystem.getInstance().visionAccurate(pose)) { // visionAccurate method sees if Apriltags present in Vision.java
      return false;
    }
    return true;
  }

  /** Sets the Odometry Pose to the given pose */
  public void setOdometryPose(Pose2d pose) {
    odometryPose = pose;
  }

  /** Returns the Odometry Pose from drivetrain */
  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  /** Sets the estimated pose to the given pose */
  public void setEstimatedPose(Pose2d pose) {
    estimatePose = pose;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometryEstimate() {
    poseEstimator.update(swerveSubsystem.getYawRotation2d(), swerveSubsystem.getSwerveModulePositions());
  }

  /**
   * @see edu.wpi.first.math.estimator.PoseEstimator#addVisionMeasurement(Pose2d, double)
   */
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  /**
   * Reset the pose estimator location and Drivetrain odometry - NEEDS TO BE TESTED
   *
   * @param poseMeters
   */
  public void resetPoseEstimate(Pose2d poseMeters) {
    swerveSubsystem.resetSwerveOdometry(poseMeters);
    poseEstimator.resetPosition(swerveSubsystem.getYawRotation2d(), swerveSubsystem.getSwerveModulePositions(), swerveSubsystem.getPose());
    
  }

  public void resetHeading(Rotation2d angle) {
    swerveSubsystem.resetSwerveOdometry(new Pose2d(swerveSubsystem.getPose().getTranslation(), angle));
    resetPoseEstimate(new Pose2d(estimatePose.getTranslation(), angle));
  }

  public void resetLocationEstimate(Translation2d translation) {
    resetPoseEstimate(new Pose2d(translation, new Rotation2d(0)));
  }

  /**
   * Gets the pose of the robot at the current time as estimated by the poseEstimator. This includes
   * vision and odometry combined together.
   *
   * @return The estimated robot pose in meters.
   */
  public Pose2d getPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Get the heading of the robot estimated by the poseEstimator. Use this in most places we would
   * use the gyro.
   *
   * @return
   */
  public Rotation2d getHeading() {
    return estimatePose.getRotation();
  }

  public Translation2d getLocation() {
    return estimatePose.getTranslation();
  }

  public Pose2d getEstimatedPose() {
    return estimatePose;
  }

  /**
   * Creates a vector of standard deviations for the states. Standard deviations of model states.
   * Increase these numbers to trust your model's state estimates less.
   *
   * @param x in meters
   * @param y in meters
   * @param theta in degrees
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N3> createStateStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }

  /**
   * Creates a vector of standard deviations for the local measurements. Standard deviations of
   * encoder and gyro rate measurements. Increase these numbers to trust sensor readings from
   * encoders and gyros less.
   *
   * @param theta in degrees per second
   * @param s std for all module positions in meters per sec
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N5> createLocalMeasurementStdDevs(double theta, double s) {
    return VecBuilder.fill(Units.degreesToRadians(theta), s, s, s, s);
  }

  /**
   * Creates a vector of standard deviations for the vision measurements. Standard deviations of
   * global measurements from vision. Increase these numbers to trust global measurements from
   * vision less.
   *
   * @param x in meters
   * @param y in meters
   * @param theta in degrees
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N3> createVisionMeasurementStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }

  /**
   * Commnad to reset odometry of drivetrain and pose esimator to the one from vision
   * @return a command to reset the Pose Estimator and Drivetrain to the vision pose
   */
  public Command resetOdometryVisionCommand(){
    return new InstantCommand(() -> resetPoseEstimate(VisionSubsystem.getInstance().visionBotPose()));
  }

  public Command tempResetOdometryCOmmand(){
    return new InstantCommand(() -> resetPoseEstimate(new Pose2d(2, 5.52, new Rotation2d(0))));
  }
  
}
