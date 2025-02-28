package org.steelhawks.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout APRIL_TAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static String[] cameraNames() {
        return switch (Constants.getRobot()) {
            case ALPHABOT ->
                new String[] {
//                    "Arducam_OV2311_USB_Camera",
                    "limelight-coral"
                };
            case HAWKRIDER ->
                new String[] {
                    "limelight-shooter",
                    "limelight"
                };
            default ->
                new String[] {
                    "arducam-front-left",
                    "arducam-front-right",
                    "arducam-back-left",
                    "arducam-elevator-mount"
                };
        };
    }

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)

    public static Transform3d[] robotToCamera() {
        return switch (Constants.getRobot()) {
            case ALPHABOT ->
                new Transform3d[] {
                    new Transform3d()
                };
            case HAWKRIDER ->
                new Transform3d[] {
                    new Transform3d(
                        Units.inchesToMeters(-7.5), 0.0, 0.2, new Rotation3d(0.0, 0.35, 0.79))
                };
            default ->
                new Transform3d[] {
                    // Front Left
                    new Transform3d(
                        Units.inchesToMeters(0),
                        Units.inchesToMeters(0),
                        Units.inchesToMeters(6.689),
                        new Rotation3d(
                            Units.degreesToRadians(0),
                            Units.degreesToRadians(0),
                            Units.degreesToRadians(0))), // Z is from the top of the belly pan

                    // Front Right
                    new Transform3d(
                        Units.inchesToMeters(10.975),
                        Units.inchesToMeters(12.556),
                        Units.inchesToMeters(6.689),
                        new Rotation3d(
                            Units.degreesToRadians(0),
                            Units.degreesToRadians(28.125),
                            Units.degreesToRadians(60))), // Z is from the top of the belly pan

                    // Back Left
                    new Transform3d(  
                        Units.inchesToMeters(- 12.556), // - 12.644
                        Units.inchesToMeters(- 10.976), // - 11.130
                        Units.inchesToMeters(6.689), // 6.783901  // Z is from the top of the belly pan
                        new Rotation3d(
                            Units.degreesToRadians(0),
                            Units.degreesToRadians(28.125),
                            Units.degreesToRadians(210))),

                    // Elevator Mount
                    new Transform3d(
                        Units.inchesToMeters(0),
                        Units.inchesToMeters(13.583),
                        Units.inchesToMeters(36.604), // Z is from the top of the belly pan
                        new Rotation3d(
                            Units.degreesToRadians(0),
                            Units.degreesToRadians(45),
                            Units.degreesToRadians(0)))
                };
        };
    }

//    public static Transform3d ROBOT_TO_CAMERA0 =
//        new Transform3d(
//            Units.inchesToMeters(-7.5), 0.0, 0.2, new Rotation3d(0.0, 0.35, 0.79));
// -7.5
    // Limelight offsets used for Vision Simulation
    public static Transform3d ROBOT_TO_CAMERA1 =
        new Transform3d(0, 0, 0, new Rotation3d());
    public static Transform3d ROBOT_TO_CAMERA2 =
        new Transform3d(0, 0, 0, new Rotation3d());

    // Basic filtering thresholds
    public static double MAX_AMBIGUITY = 0.3;
    public static double MAX_ZERROR = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
    public static double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] CAMERA_STD_DEV_FACTORS =
        switch (Constants.getRobot()) {
            case ALPHABOT ->
                new double[] {
                    3.0, // Camera 0
                };
            case HAWKRIDER ->
                new double[] {
                    1.2, // Camera 0
                    1.3 // Camera 1
                };
            default ->
                new double[] {
                    3.0, // Camera 0
                    1.2, // Camera 1
                    1.3, // Camera 2
                    1.0 // Camera 3
                };
        };

    // Multipliers to apply for MegaTag 2 observations
    public static double LINEAR_STD_DEV_MEGATAG2_FACTOR = 0.5; // More stable than full 3D solve
    public static double ANGULAR_STD_DEV_MEGATAG2_FACTOR =
        Double.POSITIVE_INFINITY; // No rotation data available
}
