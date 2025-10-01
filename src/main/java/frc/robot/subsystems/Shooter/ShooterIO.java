package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface ShooterIO extends Subsystem {
  void set(double speed);

  void setVelocity(double rpm);

  void setVoltage(double volts);

  void setEncoderPosition(double position);

  double getVelocity();

  boolean atTarget(double threshold);

  SubsystemBase returnSubsystem();
}
