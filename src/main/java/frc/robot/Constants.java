// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean kTuningMode = true;
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAutomatedSpeedMetersPerSecond = 1;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(27.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(27.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front Left
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),// Front Right
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),// Rear Left
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)// Rear Right
        );

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs

    
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 5;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 6;
    

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    // Correction Multiplies
    public static final double kDrivingMotorReduction = ((45.0 * 22) / (kDrivingMotorPinionTeeth * 15));// * 0.93;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final PathConstraints constraints = new PathConstraints(
          3, 2,
          Units.degreesToRadians(360), Units.degreesToRadians(180));
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static class VisionConstants {
    public static final String kCameraName1 = "limelight";
    public static final String kCameraName2 = "ArduR";
    public static final String kCameraName3 = "ArduL";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center,
    // pitched upward.
    private static final double camPitch1 = Units.degreesToRadians(-20);
    private static final double camYaw1 = Units.degreesToRadians(-2);
    public static final Transform3d kRobotToCam1 =
            new Transform3d(new Translation3d(Units.inchesToMeters(11), -Units.inchesToMeters(0.5), Units.inchesToMeters(7.5)), new Rotation3d(0, camPitch1, camYaw1));
    public static final Transform3d kCamToRobot1 = kRobotToCam1.inverse();

    // some of these probably need to be flipped
    private static final double camPitch2 = Units.degreesToRadians(0);
    private static final double camYaw2 = -Units.degreesToRadians(15);
    public static final Transform3d kRobotToCam2 =
            new Transform3d(new Translation3d(Units.inchesToMeters(6.25), Units.inchesToMeters(12), Units.inchesToMeters(11.75)), new Rotation3d(Math.PI, camPitch2, camYaw2));
    public static final Transform3d kCamToRobot2 = kRobotToCam2.inverse();

    // some of these probably need to be flipped
    private static final double camPitch3 = Units.degreesToRadians(0);
    private static final double camYaw3 = Units.degreesToRadians(15);
    public static final Transform3d kRobotToCam3 =
            new Transform3d(new Translation3d(Units.inchesToMeters(6.25), -Units.inchesToMeters(12), Units.inchesToMeters(11.75)), new Rotation3d(Math.PI, camPitch3, camYaw3));
    public static final Transform3d kCamToRobot3 = kRobotToCam3.inverse();


    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static class ShooterConstants {
    public static int kShooterMotorID = 9;
    
  }

  public static class AlignmentConstants {
    public static final PIDController turnPID = new PIDController(2.0, 0.0, 0.3);{turnPID.enableContinuousInput(-Math.PI, Math.PI);}

    // Tolerances
    public static final double ANGLE_TOLERANCE_RAD = Math.toRadians(5.0);
    public static final double ANG_VEL_TOLERANCE_RAD_PER_SEC = Math.toRadians(5.0);

    public static final double MAX_DIST = 5; // Meters
  }

  public static class BlinkinConstants{
    public static final double blue = 0.92;
    public static final double green = 0.73;
  }
}