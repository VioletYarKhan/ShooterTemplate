package frc.GryphonLib;

import static frc.robot.Constants.VisionConstants.kTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class PositionCalculations {
    public static Pose2d translateCoordinates(Pose2d originalPose, double degreesRotate, double distance){
        double newXCoord = originalPose.getX() + (Math.cos(Math.toRadians(degreesRotate)) * distance);
        double newYCoord = originalPose.getY() + (Math.sin(Math.toRadians(degreesRotate)) * distance);

        return new Pose2d(newXCoord, newYCoord, originalPose.getRotation());
    }


    public static Pose2d getAlignToTagPose(int tag, Transform2d transform){
        Pose2d tagPose = kTagLayout.getTagPose(tag).get().toPose2d();
        Pose2d goalPose = translateCoordinates(tagPose, tagPose.getRotation().getDegrees(), transform.getX());
        goalPose = translateCoordinates(goalPose, tagPose.getRotation().getDegrees() + 90, transform.getY());

        return goalPose.transformBy(new Transform2d(0, 0, new Rotation2d(Math.PI)));
    }

    public static Pose2d getStraightOutPose(int tag){
        Pose2d tagPose = kTagLayout.getTagPose(tag).get().toPose2d();
        Pose2d goalPose = translateCoordinates(tagPose, tagPose.getRotation().getDegrees(), 1.5);

        return goalPose.transformBy(new Transform2d(0, 0, new Rotation2d(Math.PI)));
    }
    
    public static double getYawChangeToTag(Pose2d robotPose, int tagID){
        Pose2d goalTagPose = kTagLayout.getTagPose(tagID).get().toPose2d();
        double xDiff = goalTagPose.getX() - robotPose.getX();
        double yDiff = goalTagPose.getY() - robotPose.getY();
        double angleToTag = Math.atan2(yDiff, xDiff);

        // Get robot's heading
        double robotHeading = robotPose.getRotation().getRadians();

        // Compute target angle relative to the robot
        double desiredAngle = angleToTag - robotHeading;

        // Normalize to [-π, π]
        desiredAngle = Math.atan2(Math.sin(desiredAngle), Math.cos(desiredAngle));

        double DesiredAngleToTag = 0;

        double shortestDelta = desiredAngle - DesiredAngleToTag;
        shortestDelta = Math.atan2(Math.sin(shortestDelta), Math.cos(shortestDelta));  // wrap to [-π, π]
        return shortestDelta;
    }
}
