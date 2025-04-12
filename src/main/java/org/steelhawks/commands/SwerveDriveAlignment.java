package org.steelhawks.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants.AutonConstants;
import org.steelhawks.Robot;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.SwerveDriveController;

import java.util.function.Supplier;

public class SwerveDriveAlignment extends Command {

    protected static final double XY_TOLERANCE = Units.inchesToMeters(1.0);
    protected static final double THETA_TOLERANCE = Units.degreesToRadians(5);
    protected static final double MAX_VELOCITY_ERROR_TOLERANCE = 0.15;
    protected static final double SWERVE_DEADBAND = 0.05;

    protected static final Swerve s_Swerve = RobotContainer.s_Swerve;

    protected final SwerveDriveController mController;
    protected final Supplier<Pose2d> targetPose;
    protected final Debouncer debouncer;
    protected final LinearFilter filter;
    protected Pose2d startingPose;
    protected double velocityError;

    public SwerveDriveAlignment(Pose2d targetPose) {
        this(() -> targetPose);
    }

    public SwerveDriveAlignment(Supplier<Pose2d> targetPose) {
        addRequirements(s_Swerve);
        this.targetPose = targetPose;
        this.debouncer = new Debouncer(0.2, Debouncer.DebounceType.kRising);
        this.filter = LinearFilter.movingAverage(5);

        mController =
            new SwerveDriveController(
                new PIDController(
                    AutonConstants.ALIGN_PID.kP,
                    AutonConstants.ALIGN_PID.kI,
                    AutonConstants.ALIGN_PID.kD),
                new PIDController(
                    AutonConstants.ALIGN_PID.kP,
                    AutonConstants.ALIGN_PID.kI,
                    AutonConstants.ALIGN_PID.kD),
                new ProfiledPIDController(
                    AutonConstants.ALIGN_ANGLE_PID.kP,
                    AutonConstants.ALIGN_ANGLE_PID.kI,
                    AutonConstants.ALIGN_ANGLE_PID.kD,
                    new TrapezoidProfile.Constraints(
                        AutonConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        AutonConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED)))
                .withLinearTolerance(XY_TOLERANCE)
                .withRotationalTolerance(THETA_TOLERANCE);
    }

    protected boolean isXAligned() {
        return Math.abs(targetPose.get().getX() - s_Swerve.getPose().getX()) <= XY_TOLERANCE;
    }

    protected boolean isYAligned() {
        return Math.abs(targetPose.get().getY() - s_Swerve.getPose().getY()) <= XY_TOLERANCE;
    }

    protected boolean isThetaAligned() {
        double angleDifference = MathUtil.angleModulus(targetPose.get().getRotation().getRadians() - s_Swerve.getPose().getRotation().getRadians());
        return Math.abs(angleDifference) <= THETA_TOLERANCE;
    }

    protected boolean velocityInTolerance() {
        return Math.abs(filter.calculate(velocityError)) < MAX_VELOCITY_ERROR_TOLERANCE;
    }

    protected boolean isAligned() {
        // added auton check so command keeps running if the driver wants to switch the branch to score on, this doesnt interrupt auton scoring sequence
        return isXAligned() && isYAligned() && isThetaAligned() && velocityInTolerance();
    }

    @Override
    public void initialize() {
        startingPose = s_Swerve.getPose();
        s_Swerve.setPathfinding(true);
    }

    protected ChassisSpeeds getOutput() {
        ChassisSpeeds speeds = mController.getOutput(s_Swerve.getPose(), targetPose.get());
        Logger.recordOutput("Align/ControllerOutputX", speeds.vxMetersPerSecond);
        Logger.recordOutput("Align/ControllerOutputY", speeds.vyMetersPerSecond);
        Logger.recordOutput("Align/ControllerOutputTheta", speeds.omegaRadiansPerSecond);
        return speeds;
    }

    protected void log() {
        velocityError =
            Math.hypot(
                mController.getError().vxMetersPerSecond,
                mController.getError().vyMetersPerSecond);
        Logger.recordOutput("Align/VelocityError", velocityError);

        Logger.recordOutput("Align/TargetX", targetPose.get().getX());
        Logger.recordOutput("Align/TargetY", targetPose.get().getY());
        Logger.recordOutput("Align/TargetTheta", targetPose.get().getRotation().getDegrees());
        Logger.recordOutput("Align/CurrentX", s_Swerve.getPose().getX());
        Logger.recordOutput("Align/CurrentY", s_Swerve.getPose().getY());
        Logger.recordOutput("Align/CurrentTheta", s_Swerve.getPose().getRotation().getDegrees());

        Logger.recordOutput("Align/TargetPose", targetPose.get());
        Logger.recordOutput("Align/StartingPose", startingPose);

        Logger.recordOutput("Align/AlignedX", isXAligned());
        Logger.recordOutput("Align/AlignedY", isYAligned());
        Logger.recordOutput("Align/AlignedTheta", isThetaAligned());
        Logger.recordOutput("Align/VelocityInTolerance", velocityInTolerance());
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = getOutput();
        s_Swerve.runVelocity(
            new ChassisSpeeds(
                Math.abs(speeds.vxMetersPerSecond) < SWERVE_DEADBAND ? 0 : speeds.vxMetersPerSecond,
                Math.abs(speeds.vyMetersPerSecond) < SWERVE_DEADBAND ? 0 : speeds.vyMetersPerSecond,
                Math.abs(speeds.omegaRadiansPerSecond) < SWERVE_DEADBAND ? 0 : speeds.omegaRadiansPerSecond));

        log();
    }

    @Override
    public boolean isFinished() {
        return debouncer.calculate(isAligned() && Robot.getState() == Robot.RobotState.AUTON);
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.runVelocity(new ChassisSpeeds());
        s_Swerve.setPathfinding(false);
    }
}
