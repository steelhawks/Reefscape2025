package org.steelhawks.util.autonbuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import static org.steelhawks.util.autonbuilder.BuilderConstants.*;

public enum StartEndPosition {
    NOTHING_AUTO(3, 3, 0),
    BC1(7.58, 7.2524, Units.degreesToRadians(-60.0)),
    BC2(7.000, 6.024, Units.degreesToRadians(-120)),
    BC3(7.58, 5.0812, Math.PI),

    CENTER(7.58, 4, Math.PI),

    RC1(7.58, 3, Math.PI),
    RC2(7.000, 1.907, Units.degreesToRadians(120)),
    RC3(7.58, 0.8137, Math.PI),

    TR1(5.46030758, 5.16840642, -2.0956),
    TR2(5.71848391, 5.0345549, -2.0956),

    R1(6.0003, 4.18981981 - CORAL_OFFSET, Math.PI),
    R2(6.0003, 3.857975483 - CORAL_OFFSET, Math.PI),

    BR1(4.93162509, 2.5664244, 2.101656),
    BR2(5.22050059, 2.73291654, 2.101656),

    BL1(3.24164341, 3.0350286, 1.0472),
    BL2(3.5158364, 2.86408312, 1.0472),

    L1(2.9805, 4.4952572, 0),
    L2(2.9805, 4.1633412, 0),

    TL1(3.7739639, 5.313453, -1.0472),
    TL2(4.0540139, 5.4801106, -1.0472),

    UPPER_ALGAE(1.7405211925506592, 5.857955455780029, 3.141592653589793),
    CENTER_ALGAE(1.7537007331848145, 3.999641180038452, 3.141592653589793),
    LOWER_ALGAE(1.5560076236724854, 2.1808652877807617, 3.141592653589793),
    UPPER_SOURCE(1.4526280164718628, 7.052191257476807, -0.9380478121282612),
    LOWER_SOURCE(1.3867895603179932, 1.038938045501709, 0.9463541928379318);

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