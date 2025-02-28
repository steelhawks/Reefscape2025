package org.steelhawks.autonselector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static org.steelhawks.autonselector.AutonSelectorConstants.*;

public enum StartEndPosition {
    DEFAULT_POSITION(3, 3, 0),
    BC1(7.58, 7.2524, Math.PI),
    BC2(7.58, 6.17105, Math.PI),
    BC3(7.58, 5.0812, Math.PI),

    RC1(7.58, 3, Math.PI),
    RC2(7.58, 1.9068, Math.PI),
    RC3(7.58, 0.8137, Math.PI),

    TR1(4.989901+CORAL_OFFSET_X, 5.20103502-CORAL_OFFSET_Y, -2.0956),
    TR2(5.246277332305908+CORAL_OFFSET_X, 5.067183494567871-CORAL_OFFSET_Y, -2.0956),

    R1(5.7603, 4.18981981-CORAL_OFFSET, Math.PI),
    R2(5.7603, 3.857975483-CORAL_OFFSET, Math.PI),

    BR1(4.98813963-CORAL_OFFSET_X, 2.839053-CORAL_OFFSET_Y, 2.101656),
    BR2(5.277015-CORAL_OFFSET_X, 3.00554514-CORAL_OFFSET_Y, 2.101656),

    BL1(3.71385-CORAL_OFFSET_X, 3.0024+CORAL_OFFSET_Y, 1.0472),
    BL2(3.988043-CORAL_OFFSET_X, 2.83145452+CORAL_OFFSET_Y, 1.0472),

    L1(3.2205, 4.19+CORAL_OFFSET, 0),
    L2(3.2205, 3.858084+CORAL_OFFSET, 0),

    TL1(3.71745+CORAL_OFFSET_X, 5.0408244+CORAL_OFFSET_Y, -1.0472),
    TL2(3.9975+CORAL_OFFSET_X, 5.207482+CORAL_OFFSET_Y, -1.0472),

    UPPER_ALGAE(1.7405211925506592, 5.857955455780029, 3.141592653589793),
    CENTER_ALGAE(1.7537007331848145, 3.999641180038452, 3.141592653589793),
    LOWER_ALGAE(1.5560076236724854, 2.1808652877807617, 3.141592653589793),
    UPPER_SOURCE(1.3549774885177612, 7.263299465179443, -0.9380478121282612),
    LOWER_SOURCE(1.2797433137893677, 0.8141096234321594, 0.9463541928379318);

    public final double x;
    public final double y;
    public final double rotRadians;

    StartEndPosition(double x, double y, double rotRadians) {
        this.x = x;
        this.y = y;
        this.rotRadians = rotRadians;
    }

    public Pose2d getPose() {
        return new Pose2d(x, y, new Rotation2d(rotRadians));
    }
}