package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSim extends SubsystemBase implements ShooterIO {
    private final FlywheelSim shooterSim =
        new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.04, 1), DCMotor.getKrakenX60(1), 0.005);

    private double appliedVolts = 0;
    private double targetVelocityRpm = 0;

    public ShooterSim() {}

    @Override
    public void periodic() {
        // Apply input, update sim
        shooterSim.setInputVoltage(appliedVolts);
        shooterSim.update(0.02);

        // Sim battery sag
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(shooterSim.getCurrentDrawAmps()));

        SmartDashboard.putNumber("Shooter Velocity (RPM)", getVelocity());
        SmartDashboard.putNumber("Shooter Applied Volts", appliedVolts);
    }

    @Override
    public void set(double speed) {
        appliedVolts = speed * RobotController.getBatteryVoltage();
    }

    @Override
    public void setVelocity(double rpm) {
        // In sim we approximate by setting voltage proportional to error
        targetVelocityRpm = rpm;
        double error = rpm - getVelocity();
        appliedVolts = Math.max(-12, Math.min(12, error * 0.01)); // crude P control
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
    }

    @Override
    public void setEncoderPosition(double position) {
        shooterSim.setState(VecBuilder.fill(position));
    }

    @Override
    public double getVelocity() {
        return shooterSim.getAngularVelocityRPM();
    }


    @Override
    public boolean atTarget(double threshold) {
        return Math.abs(getVelocity() - targetVelocityRpm) < threshold;
    }

    @Override
    public SubsystemBase returnSubsystem() {
        return this;
    }
}
