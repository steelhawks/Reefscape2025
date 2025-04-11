package org.steelhawks.util.autonbuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.steelhawks.ReefUtil;
import org.steelhawks.subsystems.elevator.ElevatorConstants;

import static org.steelhawks.util.autonbuilder.BuilderConstants.*;

public enum StartEndPosition {
    NOTHING_AUTO(3, 3, 0),
    BC1(7.58, 7.2524, Units.degreesToRadians(-60.0)),
    BC2(7.000, 6.024, Units.degreesToRadians(-120)),
    BC3(7.18, 5.0812, Math.PI),

    CENTER(7.085, 4, Math.PI),

    RC1(7.58, 3, Math.PI),
    RC2(7.000, 2.059, Units.degreesToRadians(120)),
    RC3(7.58, 0.8137, Math.PI),

    TR1(ReefUtil.CoralBranch.TR1.getScorePose(ElevatorConstants.State.L4).getX(),
        ReefUtil.CoralBranch.TR1.getScorePose(ElevatorConstants.State.L4).getY(),
        ReefUtil.CoralBranch.TR1.getScorePose(ElevatorConstants.State.L4).getRotation().getRadians()),

    TR2(ReefUtil.CoralBranch.TR2.getScorePose(ElevatorConstants.State.L4).getX(),
        ReefUtil.CoralBranch.TR2.getScorePose(ElevatorConstants.State.L4).getY(),
        ReefUtil.CoralBranch.TR2.getScorePose(ElevatorConstants.State.L4).getRotation().getRadians()),

    R1(ReefUtil.CoralBranch.R1.getScorePose(ElevatorConstants.State.L4).getX(),
        ReefUtil.CoralBranch.R1.getScorePose(ElevatorConstants.State.L4).getY(),
        ReefUtil.CoralBranch.R1.getScorePose(ElevatorConstants.State.L4).getRotation().getRadians()),

    R2(ReefUtil.CoralBranch.R2.getScorePose(ElevatorConstants.State.L4).getX(),
        ReefUtil.CoralBranch.R2.getScorePose(ElevatorConstants.State.L4).getY(),
        ReefUtil.CoralBranch.R2.getScorePose(ElevatorConstants.State.L4).getRotation().getRadians()),

    BR1(ReefUtil.CoralBranch.BR1.getScorePose(ElevatorConstants.State.L4).getX(),
        ReefUtil.CoralBranch.BR1.getScorePose(ElevatorConstants.State.L4).getX(),
        ReefUtil.CoralBranch.BR1.getScorePose(ElevatorConstants.State.L4).getRotation().getRadians()),

    BR2(ReefUtil.CoralBranch.BR2.getScorePose(ElevatorConstants.State.L4).getX(),
        ReefUtil.CoralBranch.BR2.getScorePose(ElevatorConstants.State.L4).getX(),
        ReefUtil.CoralBranch.BR2.getScorePose(ElevatorConstants.State.L4).getRotation().getRadians()),

    BL1(ReefUtil.CoralBranch.BL1.getScorePose(ElevatorConstants.State.L4).getX(),
        ReefUtil.CoralBranch.BL1.getScorePose(ElevatorConstants.State.L4).getY(),
        ReefUtil.CoralBranch.BL1.getScorePose(ElevatorConstants.State.L4).getRotation().getRadians()),

    BL2(ReefUtil.CoralBranch.BL2.getScorePose(ElevatorConstants.State.L4).getX(),
        ReefUtil.CoralBranch.BL2.getScorePose(ElevatorConstants.State.L4).getY(),
        ReefUtil.CoralBranch.BL2.getScorePose(ElevatorConstants.State.L4).getRotation().getRadians()),

    L1(ReefUtil.CoralBranch.L1.getScorePose(ElevatorConstants.State.L4).getX(),
        ReefUtil.CoralBranch.L1.getScorePose(ElevatorConstants.State.L4).getY(),
        ReefUtil.CoralBranch.L1.getScorePose(ElevatorConstants.State.L4).getRotation().getRadians()),
    L2(ReefUtil.CoralBranch.L2.getScorePose(ElevatorConstants.State.L4).getX(),
        ReefUtil.CoralBranch.L2.getScorePose(ElevatorConstants.State.L4).getY(),
        ReefUtil.CoralBranch.L2.getScorePose(ElevatorConstants.State.L4).getRotation().getRadians()),

    TL1(ReefUtil.CoralBranch.TL1.getScorePose(ElevatorConstants.State.L4).getX(),
        ReefUtil.CoralBranch.TL1.getScorePose(ElevatorConstants.State.L4).getY(),
        ReefUtil.CoralBranch.TL1.getScorePose(ElevatorConstants.State.L4).getRotation().getRadians()),

    TL2(ReefUtil.CoralBranch.TL2.getScorePose(ElevatorConstants.State.L4).getX(),
        ReefUtil.CoralBranch.TL2.getScorePose(ElevatorConstants.State.L4).getY(),
        ReefUtil.CoralBranch.TL2.getScorePose(ElevatorConstants.State.L4).getRotation().getRadians()),

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