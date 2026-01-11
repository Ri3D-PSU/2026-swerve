package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Translation2d;

public class Utils {
    public static double getSpeed2(ChassisSpeeds chassisSpeeds) {
        return chassisSpeeds.vxMetersPerSecond * chassisSpeeds.vxMetersPerSecond + chassisSpeeds.vyMetersPerSecond + chassisSpeeds.vyMetersPerSecond;
    }

    /**
     * Checks if a point is within a rectangular boundary.
     *
     * @param point       The Translation2d position to check.
     * @param minBoundary The bottom-left corner of the bounding box.
     * @param maxBoundary The top-right corner of the bounding box.
     * @return True if the point is inside or on the edge of the box.
     */
    public static boolean isPointInBox(Translation2d point, Translation2d minBoundary, Translation2d maxBoundary) {
        return point.getX() >= minBoundary.getX() &&
                point.getX() <= maxBoundary.getX() &&
                point.getY() >= minBoundary.getY() &&
                point.getY() <= maxBoundary.getY();
    }
}
