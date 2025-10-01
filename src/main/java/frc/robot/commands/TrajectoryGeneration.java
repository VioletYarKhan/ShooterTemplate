package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TrajectoryGeneration extends Command {
    private Pose2d goalPose;
    private DriveSubsystem m_drivetrain;
    private Field2d field;

    public TrajectoryGeneration(DriveSubsystem m_drivetrain, Pose2d goalPose, Field2d field) {
        this.m_drivetrain = m_drivetrain;
        this.goalPose = goalPose;
        this.field = field;
   }

    @Override
    public void initialize() {}   

    @Override
    public void execute() {
        field.getObject("Goal Pose").setPose(goalPose);
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(m_drivetrain.getCurrentPose());
        waypoints.add(goalPose);
        field.getObject("Current Trajectory").setPoses(waypoints);
    }
}
