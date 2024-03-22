package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;


public class ScorePositionQuad extends Command {
    private SwerveSubsystem swerveSubsystem;
    private List<Translation2d> bezierPoints;
    private PathPlannerPath path;

    public ScorePositionQuad(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
        // Create a list of bezier points from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        // This is basic default list that will be changed
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))
            );
        makePath(bezierPoints);

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. Please change asap
                new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. Set holonomic rotation.
            );

        // Prevent the path from being flipped if the coordinates are already correct (works for top left)
        path.preventFlipping = true;
    }

    public void execute() {
        AutoBuilder.followPath(path);
    }

    // First pose should get robot position from swerveOdometry then create path to speaker.
    // Method creates the path based on the quadrant that the bot is in
    public List<Translation2d> makePath(List<Translation2d> pathPoints) {
        if(swerveSubsystem.getPose().getX() < 5.90 && swerveSubsystem.getPose().getY() > 4.10) {
            pathPoints = PathPlannerPath.bezierFromPoses(
                swerveSubsystem.getPose(),
                new Pose2d(3.9, 6.0, Rotation2d.fromDegrees(0)),
                new Pose2d(1.9, 5.7, Rotation2d.fromDegrees(180))
            );
            return pathPoints;
        } else if(swerveSubsystem.getPose().getX() < 5.90 && swerveSubsystem.getPose().getY() < 4.10) {
            pathPoints = PathPlannerPath.bezierFromPoses(
                swerveSubsystem.getPose(),
                new Pose2d(3.7, 2, Rotation2d.fromDegrees(0)),
                new Pose2d(1.9, 5.7, Rotation2d.fromDegrees(180))
            );
            return pathPoints;
        } else if(swerveSubsystem.getPose().getX() < 8.30 && swerveSubsystem.getPose().getY() > 4.10) {
            pathPoints = PathPlannerPath.bezierFromPoses(
                swerveSubsystem.getPose(),
                new Pose2d(6.7, 4.55, Rotation2d.fromDegrees(0)),
                new Pose2d(5.0, 6.0, Rotation2d.fromDegrees(0)),
                new Pose2d(1.9, 5.7, Rotation2d.fromDegrees(180))
            );
            return pathPoints;
        } else {
            pathPoints = PathPlannerPath.bezierFromPoses(
                swerveSubsystem.getPose(),
                new Pose2d(6.3, 1.9, Rotation2d.fromDegrees(0)),
                new Pose2d(2.5, 2.15, Rotation2d.fromDegrees(0)),
                new Pose2d(1.9, 5.7, Rotation2d.fromDegrees(180))
            );
            return pathPoints;
        }
    }

}
