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


public class ScorePositionCircle extends Command {
    private SwerveSubsystem swerveSubsystem;
    private List<Translation2d> bezierPoints;
    private PathPlannerPath path;

    public ScorePositionCircle(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
        // Create a list of bezier points from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        // First pose should get robot position from swerveOdometry then create path to speaker.
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            swerveSubsystem.getPose(),
            new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90)) //edit line for final position
        );

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

    public void testPos() {
        if(swerveSubsystem.getPose().getX() < 1) {
            
        }
    }

}
