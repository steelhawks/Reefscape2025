package org.steelhawks.subsystems.vision;

import org.steelhawks.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout APRIL_TAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static int[] ALL_ALLOWED_TAGS = new int[] {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22
    };

    public static int[] ONLY_REEF_TAGS = new int[] {
        6, 7, 8, 9, 10, 11, 18, 19, 20, 21, 22
    };

    // Camera names, must match names configured on coprocessor
    public static String[] cameraNames() {
        return switch (Constants.getRobot()) {
            case ALPHABOT ->
                new String[] {
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
                    "arducam-center-mount"
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
                    // // FRONT LEFT (Arducam has pitch angle)
                    // new Transform3d(
                    //     // Left-Right: 11.315133
                    //     // Front-Back: 11.685378
                    //     // Up-Down: 6.689
                    //     Units.inchesToMeters(11.685378),
                    //     Units.inchesToMeters(11.315133),
                    //     Units.inchesToMeters(6.689),
                    //     new Rotation3d(
                    //         Units.degreesToRadians(0),
                    //         Units.degreesToRadians(-28.125),
                    //         Units.degreesToRadians(0))), // Z is from the top of the belly pan


                    // // FRONT LEFT (Arducam has NO pitch angle. Printed for HVR on 20250307)
                    // new Transform3d(
                    //     // Left-Right: 11.365368
                    //     // Front-Back: 12.784361
                    //     // Up-Down: 6.763611
                    //     Units.inchesToMeters(12.784361),
                    //     Units.inchesToMeters(11.365368),
                    //     Units.inchesToMeters(6.763611),
                    //     new Rotation3d(
                    //         Units.degreesToRadians(0),
                    //         Units.degreesToRadians(0),
                    //         Units.degreesToRadians(60.036534))), // Z is from the top of the belly pan

                    // // FRONT LEFT (Arducam has NO pitch angle and NO yaw angle. Printed for HVR on 20250308)
                    // new Transform3d(
                    //     // Left-Right: 11.315133
                    //     // Front-Back: 12.192615
                    //     // Up-Down: 6.657998
                    //     Units.inchesToMeters(12.192615),
                    //     Units.inchesToMeters(11.315133),
                    //     Units.inchesToMeters(6.657998),
                    //     new Rotation3d(
                    //         Units.degreesToRadians(0),
                    //         Units.degreesToRadians(0),
                    //         Units.degreesToRadians(0))), // Z is from the top of the belly pan

                    // FRONT LEFT GEARBOX MOUNT 20250316
                    new Transform3d(
                        // Left-Right: 6.841572
                        // Front-Back: 13.851222
                        // Up-Down: 10.527559
                        Units.inchesToMeters(13.851222),
                        Units.inchesToMeters(6.841572),
                        Units.inchesToMeters(10.527559),
                        new Rotation3d(
                            Units.degreesToRadians(0),
                            Units.degreesToRadians(10),
                            Units.degreesToRadians(0))), // Z is from the top of the belly pan


                    // // FRONT RIGHT (Arducam has pitch angle)
                    // new Transform3d(
                    //     // Left-Right: 10.975
                    //     // Front-Back: 12.556
                    //     // Up-Down: 6.689
                    //     Units.inchesToMeters(12.556),
                    //     Units.inchesToMeters(-10.975),
                    //     Units.inchesToMeters(6.689),
                    //     new Rotation3d(
                    //         Units.degreesToRadians(15),
                    //         Units.degreesToRadians(-28.125),
                    //         Units.degreesToRadians(-60))), // Z is from the top of the belly pan

                    // FRONT RIGHT (Arducam has NO pitch angle. Printed for HVR on 20250307)
                    new Transform3d(
                        // Left-Right: 11.559949
                        // Front-Back: 12.071904
                        // Up-Down: 6.763611
                        Units.inchesToMeters(12.071904),
                        Units.inchesToMeters(-11.559949),
                        Units.inchesToMeters(6.763611),
                        new Rotation3d(
                            Units.degreesToRadians(0),
                            Units.degreesToRadians(0),
                            Units.degreesToRadians(-30))), // Z is from the top of the belly pan

                    // // FRONT RIGHT (Arducam has NO pitch angle. Printed for HVR on 20250307)
                    // new Transform3d(
                    //     // Left-Right: 11.365372
                    //     // Front-Back: 12.783453
                    //     // Up-Down: 6.763611
                    //     Units.inchesToMeters(12.783453),
                    //     Units.inchesToMeters(-11.365372),
                    //     Units.inchesToMeters(6.763611),
                    //     new Rotation3d(
                    //         Units.degreesToRadians(0),
                    //         Units.degreesToRadians(0),
                    //         Units.degreesToRadians(-60))), // Z is from the top of the belly pan


                    // BACK LEFT
//                    new Transform3d(
//                        // Left-Right: 12.556
//                        // Front-Back: 10.976
//                        // Up-Down: 6.689
//                        Units.inchesToMeters(-10.976), // - 12.644
//                        Units.inchesToMeters(12.556), // - 11.130
//                        Units.inchesToMeters(6.689), // 6.783901  // Z is from the top of the belly pan
//                        new Rotation3d(
//                            Units.degreesToRadians(-15),
//                            Units.degreesToRadians(-28.125),
//                            Units.degreesToRadians(-210))),

                    // ELEVATOR MOUNT
//                    new Transform3d(
//                        // Left-Right: 0
//                        // Front-Back: 13.583
//                        // Up-Down: 36.604
//                        Units.inchesToMeters(13.583),
//                        Units.inchesToMeters(0),
//                        Units.inchesToMeters(36.604), // Z is from the top of the belly pan
//                        new Rotation3d(
//                            Units.degreesToRadians(0),
//                            Units.degreesToRadians(-45),
//                            Units.degreesToRadians(0))),

                    // // BRIDGE MOUNT, the arducam that's hanging off the bottom beam connecting the elevator to the superstructure
                    // new Transform3d(
                    //     // Left-Right: 2.000679
                    //     // Front-Back: 10.862403
                    //     // Up-Down: 8.312102
                    //     Units.inchesToMeters(-10.862403),
                    //     Units.inchesToMeters(-2.000679),
                    //     Units.inchesToMeters(8.312102),
                    //     new Rotation3d(
                    //         Units.degreesToRadians(8.5),
                    //         Units.degreesToRadians(-15),
                    //         Units.degreesToRadians(-150))),

                    // // BEAM MOUNT, the arducam that's hanging off of the beam supporting the funnel
                    // new Transform3d(
                    //     // Left-Right: 2.312500
                    //     // Front-Back: 14.622078
                    //     // Up-Down: 24.062500
                    //         // CHANGE DEPENDING HOW HIGH THE CAMERA IS MOUNTED ON THE BEAM
                    //         // This assumes that the camera takes up holes 7 and 8 in the beam, counted from the top
                    //     Units.inchesToMeters(-14.622078),
                    //     Units.inchesToMeters(2.312500),
                    //     Units.inchesToMeters(24.062500),
                    //     new Rotation3d(
                    //         Units.degreesToRadians(0),
                    //         Units.degreesToRadians(0),
                    //         Units.degreesToRadians(-180))),

                    // // CENTER MOUNT, the arducam that's placed on the beam inside the robot (NO PITCH)
                    // new Transform3d(
                    //     // Left-Right: 0
                    //     // Front-Back: 4.150 - 1.00 = 3.15
                    //     // Up-Down: 5.5 + 1.25 = 6.75
                    //         // CHANGE DEPENDING HOW HIGH THE CAMERA IS MOUNTED ON THE BEAM
                    //         // This assumes that the camera takes up holes 7 and 8 in the beam, counted from the top
                    //     Units.inchesToMeters(3.15),
                    //     Units.inchesToMeters(0),
                    //     Units.inchesToMeters(6.75),
                    //     new Rotation3d(
                    //         Units.degreesToRadians(0),
                    //         Units.degreesToRadians(0),
                    //         Units.degreesToRadians(0)))

                    // CENTER MOUNT, the arducam that's placed on the beam inside the robot (15 DEGREE PITCH)
                    new Transform3d(
                        // Left-Right: 0
                        // Front-Back: 2.932785
                        // Up-Down: 6.886101
                            // CHANGE DEPENDING HOW HIGH THE CAMERA IS MOUNTED ON THE BEAM
                            // This assumes that the camera takes up holes 7 and 8 in the beam, counted from the top
                        Units.inchesToMeters(2.932785),
                        Units.inchesToMeters(0),
                        Units.inchesToMeters(6.886101),
                        new Rotation3d(
                            Units.degreesToRadians(0),
                            Units.degreesToRadians(-15),
                            Units.degreesToRadians(0)))

                    // // CENTER RIGHT MOUNT, the arducam that's placed on the right side of the beam inside the robot (30 DEGREE YAW)
                    // new Transform3d(
                    //     // Left-Right: 9.062338
                    //     // Front-Back: 3.134462
                    //     // Up-Down: 6.900835
                    //         // CHANGE DEPENDING HOW HIGH THE CAMERA IS MOUNTED ON THE BEAM
                    //         // This assumes that the camera takes up holes 7 and 8 in the beam, counted from the top
                    //     Units.inchesToMeters(3.134462),
                    //     Units.inchesToMeters(-9.062338),
                    //     Units.inchesToMeters(6.900835),
                    //     new Rotation3d(
                    //         Units.degreesToRadians(0),
                    //         Units.degreesToRadians(-15),
                    //         Units.degreesToRadians(30)))
                    

                    // Hello This Is Rahman Arssath FRC 2601 2025
                };
        };
    }

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
                    3.0, // arducam-front-left
                    1.5, // arducam-front-right
                    1.3 // arducam-center-mount
                };
        };

    // Multipliers to apply for MegaTag 2 observations
    public static double LINEAR_STD_DEV_MEGATAG2_FACTOR = 0.5; // More stable than full 3D solve
    public static double ANGULAR_STD_DEV_MEGATAG2_FACTOR =
        Double.POSITIVE_INFINITY; // No rotation data available
}
