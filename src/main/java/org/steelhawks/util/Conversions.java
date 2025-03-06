package org.steelhawks.util;

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
}
