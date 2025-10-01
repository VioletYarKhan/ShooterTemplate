// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlignToGoal;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterSim;
import frc.robot.subsystems.Shooter.ShooterTalonFX;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  // Subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ShooterIO m_shooter = Robot.isReal() ? new ShooterTalonFX() : new ShooterSim();
  // private final IntakeSubsystem m_intake = new IntakeSubsystem();

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  // Robot state machine
  public enum State {
    NoPiece,
    StowedPiece,
    ReadyForShoot,
    Shooting,
    Intaking,
    Jammed
  }

  private State currentState = State.StowedPiece;

  public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindings();
    configureStateTriggers();
  }

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(
        new RunCommand(
            () -> {
              double forward = -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband);
              double strafe = -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband);
              double turn = -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband);

              m_drive.drive(forward, strafe, turn, true);
            },
            m_drive));
  }

  private void configureButtonBindings() {
    // Driver bindings
    m_driverController.rightBumper().whileTrue(new AlignToGoal(m_drive, m_driverController, DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7));
    m_driverController.rightTrigger().whileTrue(
      new RunCommand(()->{
        if (currentState == State.ReadyForShoot){
          currentState = State.Shooting;
          // TODO: feed passthrough/intake to spun-up shooter
        }
      }, m_shooter)
    );

    // Operator bindings
    m_operatorController.start().onTrue(new InstantCommand(m_drive::zeroHeading, m_drive));
    m_operatorController.a().onTrue(new InstantCommand(m_drive::stop, m_drive));
    m_operatorController.y().onTrue(new InstantCommand(m_drive::setX, m_drive));

    // Shooter manual
    m_operatorController.rightTrigger().whileTrue(
        new RunCommand(() -> m_shooter.set(1.0), m_shooter)).onFalse(
          new InstantCommand(()->m_shooter.set(0), m_shooter));

  }

  private void configureStateTriggers() {
    Trigger aligned = new Trigger(m_drive::getAligned);
    Trigger inRange =
        new Trigger(() -> m_drive.getCurrentPose().getTranslation().getDistance(
            VisionConstants.kTagLayout
                .getTagPose(DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7)
                .get().toPose2d().getTranslation())
            < AlignmentConstants.MAX_DIST);

    // TODO: Replace with real sensor for piece detection
    Trigger hasPiece = new Trigger(() -> true);

    hasPiece.onFalse(new InstantCommand(()->currentState = State.NoPiece)).onTrue(new InstantCommand(()->currentState = State.StowedPiece));

    Trigger SpunUp = new Trigger(() -> m_shooter.getVelocity() > 500);

    (inRange.and(hasPiece)).whileTrue(new RunCommand(() -> m_shooter.setVelocity(800), m_shooter)).onFalse(new RunCommand(() -> m_shooter.set(0), m_shooter));


    Trigger readyToShoot = new Trigger(aligned.and(inRange).and(SpunUp).and(hasPiece));

    readyToShoot.onTrue(new InstantCommand(() -> currentState = State.ReadyForShoot));
  }

  /** Returns the autonomous command. */
  public SequentialCommandGroup getAutonomousCommand() {
    // TODO: Fill in with auto
    return new SequentialCommandGroup();
  }

  /** Returns current robot state. */
  public State getCurrentState() {
    return currentState;
  }
}
