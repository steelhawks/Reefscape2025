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
