package org.steelhawks.util;

import edu.wpi.first.math.geometry.Pose3d;

public record AprilTag(int id, Pose3d pose) {
    public static int tagToArrayIndex(int tag) {
        return tag - 1;
    }
}
