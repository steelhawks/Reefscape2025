package org.steelhawks.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class FusedSwerveDriveAlignment extends SwerveDriveAlignment {

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    public FusedSwerveDriveAlignment(Pose2d targetPose, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this(() -> targetPose, xSupplier, ySupplier);
    }

    public FusedSwerveDriveAlignment(Supplier<Pose2d> targetPose, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        super(targetPose);
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = super.getOutput();
        s_Swerve.runVelocity(
            new ChassisSpeeds(
                Math.abs(speeds.vxMetersPerSecond) < SWERVE_DEADBAND ? 0 : speeds.vxMetersPerSecond,
                Math.abs(speeds.vyMetersPerSecond) < SWERVE_DEADBAND ? 0 : speeds.vyMetersPerSecond,
                Math.abs(speeds.omegaRadiansPerSecond) < SWERVE_DEADBAND ? 0 : speeds.omegaRadiansPerSecond)
                .plus(
                    new ChassisSpeeds(
                        xSupplier.getAsDouble(),
                        ySupplier.getAsDouble(),
                        0)));
        super.log();
    }
}
