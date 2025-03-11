package org.steelhawks.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.Constants.AutonConstants;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.AllianceFlip;

import java.util.function.Supplier;

public class SwerveDriveAlignment extends Command {

    private static final double X_TOLERANCE = 0.1;
    private static final double Y_TOLERANCE = 0.1;
    private static final double THETA_TOLERANCE = 5;

    private static final Swerve s_Swerve = RobotContainer.s_Swerve;

    private final HolonomicDriveController mController;
    private final Supplier<Pose2d> targetPose;
    private Pose2d startingPose;

    public SwerveDriveAlignment(Supplier<Pose2d> targetPose) {
        addRequirements(s_Swerve);
        this.targetPose = targetPose;

        mController = new HolonomicDriveController(
            new PIDController(
                AutonConstants.TRANSLATION_KP,
                AutonConstants.TRANSLATION_KI,
                AutonConstants.TRANSLATION_KD),
            new PIDController(
                AutonConstants.TRANSLATION_KP,
                AutonConstants.TRANSLATION_KI,
                AutonConstants.TRANSLATION_KD),
            new ProfiledPIDController(
                AutonConstants.ROTATION_KP,
                AutonConstants.ROTATION_KI,
                AutonConstants.ROTATION_KD,
                new TrapezoidProfile.Constraints(
                    AutonConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    AutonConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED)));
    }

    private boolean isXAligned() {
        return Math.abs(targetPose.get().getX() - s_Swerve.getPose().getX()) < X_TOLERANCE;
    }

    private boolean isYAligned() {
        return Math.abs(targetPose.get().getY() - s_Swerve.getPose().getY()) < Y_TOLERANCE;
    }

    private boolean isThetaAligned() {
        return Math.abs(targetPose.get().getRotation().getDegrees() - s_Swerve.getPose().getRotation().getDegrees()) < THETA_TOLERANCE;
    }

    @Override
    public void initialize() {
        startingPose = s_Swerve.getPose();
    }

    @Override
    public void execute() {
        s_Swerve.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                mController.calculate(s_Swerve.getPose(), targetPose.get(), AutonConstants.MAX_VELOCITY_METERS_PER_SECOND, targetPose.get().getRotation()),
                AllianceFlip.shouldFlip()
                    ? s_Swerve.getRotation().plus(new Rotation2d(Math.PI))
                    : s_Swerve.getRotation()));

        Logger.recordOutput("Align/TargetX", targetPose.get().getX());
        Logger.recordOutput("Align/TargetY", targetPose.get().getY());
        Logger.recordOutput("Align/TargetTheta", targetPose.get().getRotation().getDegrees());
        Logger.recordOutput("Align/CurrentX", s_Swerve.getPose().getX());
        Logger.recordOutput("Align/CurrentY", s_Swerve.getPose().getY());
        Logger.recordOutput("Align/CurrentTheta", s_Swerve.getPose().getRotation().getDegrees());

        Logger.recordOutput("Align/TargetPose", targetPose.get());
        Logger.recordOutput("Align/CurrentPose", s_Swerve.getPose());
        Logger.recordOutput("Align/StartingPose", startingPose);

        Logger.recordOutput("Align/AlignedX", isXAligned());
        Logger.recordOutput("Align/AlignedY", isYAligned());
        Logger.recordOutput("Align/AlignedTheta", isThetaAligned());
    }

    @Override
    public boolean isFinished() {
        return isXAligned() && isYAligned() && isThetaAligned();
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.runVelocity(new ChassisSpeeds());
        s_Swerve.stopWithX();
    }
}
