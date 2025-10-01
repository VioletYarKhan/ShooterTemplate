package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterTalonFX extends SubsystemBase implements ShooterIO {
  private final TalonFX shooterMotor = new TalonFX(ShooterConstants.kShooterMotorID);
  private TalonFXConfiguration shooterConfigurator = new TalonFXConfiguration();

  private double targetReference = 0;
  private ControlType currentControlType = ControlType.kPosition;

  public ShooterTalonFX() {
    shooterConfigurator.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterConfigurator.Slot0.kP = 1.0;

    shooterMotor.getConfigurator().apply(shooterConfigurator);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Velocity (RPM)", getVelocity());
  }

  @Override
  public void set(double speed) {
    shooterMotor.set(speed);
  }

  @Override
  public void setVelocity(double rpm) {
    targetReference = rpm;
    shooterMotor.setControl(new VelocityDutyCycle(rpm));
    currentControlType = ControlType.kVelocity;
  }

  public void setPosition(double position){
    SmartDashboard.putNumber("Requested Wheel Position", position);
    shooterMotor.setControl(new PositionDutyCycle(position));
    
    targetReference = position;
    currentControlType = ControlType.kPosition;
}

  @Override
  public void setVoltage(double volts) {
    shooterMotor.setVoltage(volts);
  }

  @Override
  public void setEncoderPosition(double position) {
    shooterMotor.setPosition(position);
  }

  @Override
  public double getVelocity() {
    // convert to RPM if needed
    return shooterMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public boolean atTarget(double threshold) {
    if (currentControlType == ControlType.kVelocity) {
        return Math.abs(getVelocity() - targetReference) < threshold;
    } else {
        return false;
    }
  }

  @Override
  public SubsystemBase returnSubsystem() {
    return this;
  }
}
