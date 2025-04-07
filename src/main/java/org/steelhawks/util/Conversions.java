package org.steelhawks.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.geometry.Vector3;

public class Conversions {

    /**
     * @param angle angle in degrees
     * @return angle in the range of 0 to 360
     */
    public static double continuous180To360(double angle) {
        return (angle + 360) % 360;
    }

    /**
     * @param angle angle in radians
     * @return returns a continuous angle from 0-2pi to an angle that is -pi to pi
     */
    public static double convert360To180Rad(double angle) {
        return (angle + Math.PI) % (2 * Math.PI) - Math.PI;
    }

    public static double convert360To180(double angle) {
        return (angle + 180) % 360 - 180;
    }

    /**
     * @param wheelRPS      Wheel Velocity: (in Rotations per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Meters per Second)
     */
    public static double RPSToMPS(double wheelRPS, double circumference) {
        return wheelRPS * circumference;
    }

    /**
     * @param wheelMPS      Wheel Velocity: (in Meters per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Rotations per Second)
     */
    public static double MPSToRPS(double wheelMPS, double circumference) {
        return wheelMPS / circumference;
    }

    /**
     * @param wheelRotations Wheel Position: (in Rotations)
     * @param circumference  Wheel Circumference: (in Meters)
     * @return Wheel Distance: (in Meters)
     */
    public static double rotationsToMeters(double wheelRotations, double circumference) {
        return wheelRotations * circumference;
    }

    /**
     * @param wheelMeters   Wheel Distance: (in Meters)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Position: (in Rotations)
     */
    public static double metersToRotations(double wheelMeters, double circumference) {
        return wheelMeters / circumference;
    }

    /**
     * @param translation   Translation2d to convert to a Vector2
     * @return Converted Vector2
     */
    public static Vector2 toVector2(Translation2d translation) {
        return new Vector2(translation.getX(), translation.getY());
    }

    /**
     * @param pose   Pose2d to convert to a Vector2
     * @return Converted Vector2
     */
    public static Vector2 toVector2(Pose2d pose) {
        return toVector2(pose.getTranslation());
    }

    public static Translation2d toTranslation2d(Vector2 vector) {
        return new Translation2d(vector.x, vector.y);
    }

    public static Translation2d[] toTranslation2dArray(Vector2[] vector) {
        Translation2d[] translation = new Translation2d[vector.length];
        for (int i = 0; i < vector.length; i++) {
            translation[i] = new Translation2d(vector[i].x, vector[i].y);
        }
        return translation;
    }

    /**
     * @param translation   Translation3d to convert to a Vector3
     * @return Converted Vector3
     */
    public static Vector3 toVector3(Translation3d translation) {
        return new Vector3(translation.getX(), translation.getY(), translation.getZ());
    }

    /**
     * @param translation   Pose3d to convert to a Vector3
     * @return Converted Vector3
     */
    public static Vector3 toVector3(Pose3d translation) {
        return toVector3(translation.getTranslation());
    }
}
