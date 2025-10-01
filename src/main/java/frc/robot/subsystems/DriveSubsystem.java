// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Vision;
import frc.GryphonLib.MovementCalculations;
import frc.GryphonLib.PositionCalculations;
import frc.littletonUtils.PoseEstimator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.TrajectoryGeneration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private double gyroOffset = 0.0;

  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  // private static Vector<N3> LLStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(10));
  // private static Matrix<N3, N1> LLstdevsMat = new Matrix<>(LLStdDevs.getStorage());
  private static Vector<N3> ArduStdDevs = VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(10));
  private static Matrix<N3, N1> ArdustdevsMat = new Matrix<>(ArduStdDevs.getStorage());
  private final PoseEstimator poseEstimator;
  private final Field2d field2d = new Field2d();
  private final StructArrayPublisher<SwerveModuleState> publisher;
  private double currentTimestamp = Timer.getTimestamp();
  private boolean aligned = false;



  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
    var alliance = DriverStation.getAlliance();

    poseEstimator = new PoseEstimator(stateStdDevs);
    
    Logger.recordOutput("Robot Pose", getCurrentPose());
    Logger.recordOutput("Goal Pose", field2d.getObject("Goal Pose").getPose());
    Logger.recordOutput("Current Trajectory", field2d.getObject("Current Trajectory").getPose());

    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty("Front Left Angle", ()->m_frontLeft.getState().angle.getDegrees(), null);
          builder.addDoubleProperty("Front Left Velocity", ()->m_frontLeft.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty("Front Right Angle", ()->m_frontRight.getState().angle.getDegrees(), null);
          builder.addDoubleProperty("Front Right Velocity", ()->m_frontRight.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty("Back Left Angle", ()->m_rearLeft.getState().angle.getDegrees(), null);
          builder.addDoubleProperty("Back Left Velocity", ()->m_rearLeft.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty("Back Right Angle", ()->m_rearRight.getState().angle.getDegrees(), null);
          builder.addDoubleProperty("Back Right Velocity", ()->m_rearRight.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty("Robot Angle", ()->getRotation().getRadians(), null);
      }
    });

    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      return;
    }
    AutoBuilder.configure(
      this::getCurrentPose, // Robot pose supplier
      this::setCurrentPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> driveRobotRelativeChassis(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
      ),
      config, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );

    setpointGenerator = new SwerveSetpointGenerator(
        config, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
        Units.rotationsToRadians(10.0) // The max rotation velocity of a swerve module in radians per second. This should probably be stored in your Constants file
    );

    ChassisSpeeds currentSpeeds = getCurrentSpeeds(); // Method to get current robot-relative chassis speeds
    SwerveModuleState[] currentStates = getStates(); // Method to get the current swerve module states
    previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(config.numModules));
  }

  public void driveRobotRelativeChassis(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void driveWithSetpoints(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var deliveredSpeeds = fieldRelative
    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
      Rotation2d.fromDegrees(Robot.isReal() ? getHeading() : getCurrentPose().getRotation().getDegrees()))
      : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    /*
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(deliveredSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
    swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    */

      previousSetpoint = setpointGenerator.generateSetpoint(
        previousSetpoint, // The previous setpoint
        deliveredSpeeds, // The desired target speeds
        0.02 // The loop time of the robot code, in seconds
      );

    var swerveModuleStates = previousSetpoint.moduleStates();

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (Robot.isSimulation()){
      driveWithSetpoints(xSpeed, ySpeed, rot, fieldRelative);
      return;
    }
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var deliveredSpeeds = fieldRelative
    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
      Rotation2d.fromDegrees(Robot.isReal() ? getHeading() : getCurrentPose().getRotation().getDegrees()))
      : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(deliveredSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
    swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyroOffset = -m_gyro.getAngle(); // Set current yaw as zero
  }

  public void setHeading(double angle) {
    gyroOffset = (angle - m_gyro.getAngle()); 
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getAngle(IMUAxis.kZ) + gyroOffset;
  }

  /**
   * Returns the current chassis speeds of the robot
   * @return the robot's current speeds
   */
  public ChassisSpeeds getCurrentSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] modules = {
    m_frontLeft.getPosition(),
    m_frontRight.getPosition(),
    m_rearLeft.getPosition(),
    m_rearRight.getPosition()};

    return modules;
  }


  public SwerveModuleState[] getStates(){
    SwerveModuleState[] modules = {
    m_frontLeft.getState(),
    m_frontRight.getState(),
    m_rearLeft.getState(),
    m_rearRight.getState()};

    return modules;
  }

  public Rotation2d getRotation(){
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ) + gyroOffset);
  }

  public void stop(){
    driveRobotRelativeChassis(new ChassisSpeeds());
  }

  public PathPlannerPath getPathFromWaypoint(Pose2d waypoint) {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        getCurrentPose(),
        waypoint
    );
    PathPlannerPath path = new PathPlannerPath(
      waypoints, 
      AutoConstants.constraints,
      new IdealStartingState(MovementCalculations.getVelocityMagnitude(getCurrentSpeeds()), getRotation()), 
      new GoalEndState(0.0, waypoint.getRotation())
    );
    return path;
  }

  public Command followPath(PathPlannerPath path){
    return AutoBuilder.followPath(path);
  }

  public Command PathToPose(Pose2d goalPose, double endSpeed){
    field2d.getObject("Goal Pose").setPose(goalPose);
    List<Pose2d> waypoints = List.of(getCurrentPose(), goalPose);
    field2d.getObject("Current Trajectory").setPoses(waypoints);

    Command pathfindingCommand = AutoBuilder.pathfindToPose(
        goalPose,
        AutoConstants.constraints,
        endSpeed // Goal end velocity in meters/sec
    );

    return new ParallelRaceGroup(pathfindingCommand, new TrajectoryGeneration(this, goalPose, field2d));
  }

  public Command AlignToTagFar(int goalTag){
    Pose2d goalPose;
    if (goalTag == 0){
      goalPose = getCurrentPose();
    } else {
      goalPose = PositionCalculations.getStraightOutPose(goalTag);
    }
    return PathToPose(goalPose, 0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distance to Goal (m)", getDistanceToPose(VisionConstants.kTagLayout.getTagPose(DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7).get().toPose2d()));
    SmartDashboard.putBoolean("Aligned to Goal", aligned);
    if (Vision.getResult1() != null){
      Optional<EstimatedRobotPose> visionBotPose1 = Vision.getEstimatedGlobalPoseCam1(getCurrentPose(), Vision.getResult1());
      if (visionBotPose1.isPresent()){
        // poseEstimator.addVisionData(List.of(visionBotPose1.get()), LLstdevsMat);
        field2d.getObject("Camera1 Pose Guess").setPose(visionBotPose1.get().estimatedPose.toPose2d());
      }
    }
    if (Vision.getResult2() != null){
      Optional<EstimatedRobotPose> visionBotPose2 = Vision.getEstimatedGlobalPoseCam2(getCurrentPose(), Vision.getResult2());
      if (visionBotPose2.isPresent()){
        if(visionBotPose2.get().targetsUsed.size() > 1){
          poseEstimator.addVisionData(List.of(visionBotPose2.get()), VisionConstants.kMultiTagStdDevs);
        } else{
          poseEstimator.addVisionData(List.of(visionBotPose2.get()), ArdustdevsMat);
        }
        field2d.getObject("Camera2 Pose Guess").setPose(visionBotPose2.get().estimatedPose.toPose2d());
      }
    }
    if (Vision.getResult3() != null){
      Optional<EstimatedRobotPose> visionBotPose3 = Vision.getEstimatedGlobalPoseCam3(getCurrentPose(), Vision.getResult3());
      if (visionBotPose3.isPresent()){
        if(visionBotPose3.get().targetsUsed.size() > 1){
          poseEstimator.addVisionData(List.of(visionBotPose3.get()), VisionConstants.kMultiTagStdDevs);
        }
        else{
          poseEstimator.addVisionData(List.of(visionBotPose3.get()), ArdustdevsMat);
        }
        field2d.getObject("Camera3 Pose Guess").setPose(visionBotPose3.get().estimatedPose.toPose2d());
      }
    }
    
    // Update pose estimator with drivetrain sensors
    poseEstimator.addDriveData(
      Timer.getTimestamp(),
      getCurrentSpeeds().toTwist2d(Timer.getTimestamp() - currentTimestamp)
      );

      field2d.setRobotPose(getCurrentPose());
      publisher.set(getStates());
    SmartDashboard.putData("Field", field2d);
    SmartDashboard.putNumber("Current Speed", MovementCalculations.getVelocityMagnitude(getCurrentSpeeds()).magnitude());
    currentTimestamp = Timer.getTimestamp();
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getLatestPose();
  }

  public double getDistanceToGoal(){
    return PhotonUtils.getDistanceToPose(getCurrentPose(), field2d.getObject("Goal Pose").getPose());
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPose(newPose);
  }

  public void setAlign(boolean alignedNow) {
    aligned = alignedNow;
  }

  public boolean getAligned(){
    return aligned;
  }

  public double getDistanceToPose(Pose2d pose){
    return getCurrentPose().getTranslation().getDistance(pose.getTranslation());
  }
}