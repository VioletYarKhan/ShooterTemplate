package frc.GryphonLib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PosititionalKraken extends SubsystemBase {
    
    TalonFX krakenMotor = new TalonFX(13);
    TalonFXConfiguration krakenConfig = new TalonFXConfiguration();

    double targetReference;
    ControlType currentControlType;

    public PosititionalKraken() {
        krakenConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        krakenConfig.ClosedLoopRamps.withDutyCycleClosedLoopRampPeriod(0).withTorqueClosedLoopRampPeriod(0).withVoltageClosedLoopRampPeriod(0);
        krakenConfig.OpenLoopRamps.withDutyCycleOpenLoopRampPeriod(0).withTorqueOpenLoopRampPeriod(0).withVoltageOpenLoopRampPeriod(0);
        krakenConfig.Slot0.withKP(0.05);
                    
        krakenMotor.getConfigurator().apply(krakenConfig);


        targetReference = 0;
        currentControlType = ControlType.kDutyCycle;
    }

    public void set(double speed) {
        krakenMotor.set(speed);
        currentControlType = ControlType.kDutyCycle;
    }

    public void setVelocity(double velocity) {
        krakenMotor.setControl(new VelocityDutyCycle(velocity));
        targetReference = velocity;
        currentControlType = ControlType.kVelocity;
    }

    public void setPosition(double position) {
        krakenMotor.setControl(new PositionDutyCycle(position));
        targetReference = position;
        currentControlType = ControlType.kPosition;
    }

    public void setVoltage(double voltage) {
        krakenMotor.setVoltage(voltage);
        currentControlType = ControlType.kVoltage;
    }

    public void setEncoderPosition(double position) {
        krakenMotor.setPosition(position);
    }

    public double getVelocity() {
        return krakenMotor.getVelocity().getValueAsDouble();
    }

    public double getPosition() {
        return krakenMotor.getPosition().getValueAsDouble();
    }

    public boolean atTarget(double threshold) {
        if (currentControlType == ControlType.kVelocity) {
            return Math.abs(getVelocity() - targetReference) < threshold;
        } else if (currentControlType == ControlType.kPosition) {
            return Math.abs(getPosition() - targetReference) < threshold;
        } else {
            return false;
        }
    }
}
