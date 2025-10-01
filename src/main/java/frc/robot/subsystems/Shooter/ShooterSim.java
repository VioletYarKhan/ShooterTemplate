package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSim extends SubsystemBase implements ShooterIO {
    private final FlywheelSim shooterSim =
        new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(2), 0.04, 1), DCMotor.getKrakenX60(1), 0.005);

    private double appliedVolts = 0;
    private double targetVelocityRpm = 0;
    private double wheelAngle = 0.0;

    private final Mechanism2d mech2d = new Mechanism2d(0.5, 0.5); // 0.5 m square
    private final MechanismRoot2d root = mech2d.getRoot("ShooterBase", 0.25, 0.25);
    private final MechanismLigament2d wheelVisual = root.append(new MechanismLigament2d("Wheel", 0.2, 0));
    

    public ShooterSim() {}

    @Override
    public void periodic() {
        shooterSim.setInputVoltage(appliedVolts);
        shooterSim.update(0.02);
        double velocityRPM = getVelocity();

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(shooterSim.getCurrentDrawAmps()));

        wheelAngle += Units.rotationsToDegrees(velocityRPM * 0.02 / 60.0);
        wheelAngle %= 360;
        wheelVisual.setAngle(wheelAngle);


        SmartDashboard.putData("Shooter Mech", mech2d);
        SmartDashboard.putNumber("Shooter Velocity (RPM)", velocityRPM);
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
