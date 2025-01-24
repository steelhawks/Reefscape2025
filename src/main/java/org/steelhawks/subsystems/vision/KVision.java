package org.steelhawks.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public final class KVision {

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final int LATENCY_WAIT_TIME = 250;

    public static final String CAM_01_NAME = "camera_0";
    public static final String CAM_02_NAME = "camera_1";

    // If using a Limelight, configure on Web GUI.
    public static final Transform3d CAM_01_TO_ROBOT = new Transform3d();
    public static final Transform3d CAM_02_TO_ROBOT = new Transform3d();

    // Basic Filter Thresholds
    public static final double MAX_AMBIGUITY = 0.3;
    public static final double MAX_Z_ERROR = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
    public static double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] CAM_STD_DEV_FACTORS = new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double LINEAR_STD_DEV_MEGATAG2_FACTOR = 0.5; // More stable than full 3D solve
    public static double ANGULAR_STD_DEV_MEGATAG2_FACTOR = Double.POSITIVE_INFINITY; // No rotation data available
}
