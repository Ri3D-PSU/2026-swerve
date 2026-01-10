package frc.robot;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Drive;

import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Future;

public class AsyncPathGenerator {
    public static Future<PathPlannerPath> generatePathAsync(Pose2d finalPosition, Rotation2d goalRotation, Drive drive) {
        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        var fieldRelativeSpeeds = drive.getFieldRelativeSpeeds();
        var robotPose = drive.getPose();
        Rotation2d splineRotation;

        if (Math.abs(fieldRelativeSpeeds.vxMetersPerSecond) + Math.abs(fieldRelativeSpeeds.vyMetersPerSecond) < 0.01) {
            splineRotation = new Rotation2d(finalPosition.getX() - robotPose.getX(), finalPosition.getY() - robotPose.getY());
        } else {
            splineRotation = new Rotation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
        }


        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(drive.getPose().getTranslation(), splineRotation),
                finalPosition
        );

        return CompletableFuture.supplyAsync(() -> {
            PathConstraints constraints = new PathConstraints(Constants.MAX_LINEAR_SPEED, 2.0, Constants.MAX_ANGULAR_SPEED, 4 * Math.PI); // The constraints for this path.
            // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

            // Create the path using the waypoints created above
            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                    new GoalEndState(0.0, goalRotation) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            return path;
        });
    }
}
