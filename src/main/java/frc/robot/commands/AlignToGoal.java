package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.GryphonLib.PositionCalculations;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToGoal extends Command {
    private final DriveSubsystem drive;
    private final PIDController turnPID = AlignmentConstants.turnPID;
    private final CommandXboxController controller;
    private double yawError;
    private Debouncer alignDebouncer = new Debouncer(0.05);

    public AlignToGoal(DriveSubsystem drive, CommandXboxController controller, int tagID) {
        this.drive = drive;
        this.controller = controller;

        addRequirements(drive);
        yawError = PositionCalculations.getYawChangeToTag(drive.getCurrentPose(), tagID);
    }

    @Override
    public void initialize() {
        turnPID.reset();
    }

    @Override
    public void execute() {
        // Driver translation inputs
        double forward = -MathUtil.applyDeadband(controller.getLeftY(), OIConstants.kDriveDeadband);
        double strafe  = -MathUtil.applyDeadband(controller.getLeftX(), OIConstants.kDriveDeadband);

        // Compute yaw error based on alliance
        yawError = PositionCalculations.getYawChangeToTag(
            drive.getCurrentPose(),
            DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7
        );
        SmartDashboard.putNumber("Yaw Align Error", Units.degreesToRadians(yawError));

        // PID output
        double turn = turnPID.calculate(yawError, 0.0);
        turn = MathUtil.clamp(turn, -1.0, 1.0);

        // Drive with driver’s translation + auto-turn
        drive.drive(forward, strafe, -turn, true);
        boolean withinAngleTol = Math.abs(yawError) < AlignmentConstants.ANGLE_TOLERANCE_RAD;
        boolean slowEnough = Math.abs(drive.getCurrentSpeeds().omegaRadiansPerSecond) < AlignmentConstants.ANG_VEL_TOLERANCE_RAD_PER_SEC;

        // Debouncer ensures it's stable for required time
        boolean alignedNow = alignDebouncer.calculate(withinAngleTol && slowEnough);
        drive.setAlign(alignedNow);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0.0, 0.0, 0.0, true);
        drive.setAlign(false);
    }
}
