package org.steelhawks.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
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

    private static final double XY_TOLERANCE = Units.inchesToMeters(1.0);
    private static final double THETA_TOLERANCE = Units.degreesToRadians(5);
    private static final double MAX_VELOCITY_ERROR_TOLERANCE = 0.15;

    private static final Swerve s_Swerve = RobotContainer.s_Swerve;

//    private final HolonomicDriveController mController;
    private final SwerveDriveController mController;
    private final Supplier<Pose2d> targetPose;
    private final Debouncer debouncer;
    private Pose2d startingPose;
    private double velocityError;

    public SwerveDriveAlignment(Supplier<Pose2d> targetPose) {
        addRequirements(s_Swerve);
        this.targetPose = targetPose;
        this.debouncer = new Debouncer(0.2, Debouncer.DebounceType.kRising);

//        mController = new HolonomicDriveController(
//            new PIDController(
//                AutonConstants.TRANSLATION_PID.kP,
//                AutonConstants.TRANSLATION_PID.kI,
//                AutonConstants.TRANSLATION_PID.kD),
//            new PIDController(
//                AutonConstants.TRANSLATION_PID.kP,
//                AutonConstants.TRANSLATION_PID.kI,
//                AutonConstants.TRANSLATION_PID.kD),
//            new ProfiledPIDController(
//                AutonConstants.ROTATION_PID.kP,
//                AutonConstants.ROTATION_PID.kI,
//                AutonConstants.ROTATION_PID.kD,
//                new TrapezoidProfile.Constraints(
//                    AutonConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
//                    AutonConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED)));
//        mController.getXController().setTolerance(XY_TOLERANCE);
//        mController.getYController().setTolerance(XY_TOLERANCE);
//        mController.getThetaController().setTolerance(THETA_TOLERANCE);
        mController =
            new SwerveDriveController(
                new PIDController(
                    AutonConstants.TRANSLATION_PID.kP,
                    AutonConstants.TRANSLATION_PID.kI,
                    AutonConstants.TRANSLATION_PID.kD),
                new PIDController(
                    AutonConstants.ROTATION_PID.kP,
                    AutonConstants.ROTATION_PID.kI,
                    AutonConstants.ROTATION_PID.kD),
                new ProfiledPIDController(
                    AutonConstants.ROTATION_PID.kP,
                    AutonConstants.ROTATION_PID.kI,
                    AutonConstants.ROTATION_PID.kD,
                    new TrapezoidProfile.Constraints(
                        AutonConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        AutonConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED)))
            .withLinearTolerance(XY_TOLERANCE)
            .withRotationalTolerance(THETA_TOLERANCE);
    }

    private boolean isXAligned() {
        return Math.abs(targetPose.get().getX() - s_Swerve.getPose().getX()) < XY_TOLERANCE;
    }

    private boolean isYAligned() {
        return Math.abs(targetPose.get().getY() - s_Swerve.getPose().getY()) < XY_TOLERANCE;
    }

    private boolean isThetaAligned() {
        return Math.abs(targetPose.get().getRotation().getRadians() - s_Swerve.getPose().getRotation().getRadians()) < THETA_TOLERANCE;
    }

    private boolean isAligned() {
        // added auton check so command keeps running if the driver wants to switch the branch to score on, this doesnt interrupt auton scoring sequence
        return isXAligned() && isYAligned() && isThetaAligned() && velocityError < MAX_VELOCITY_ERROR_TOLERANCE && Robot.getState() == Robot.RobotState.AUTON;
    }

    @Override
    public void initialize() {
        startingPose = s_Swerve.getPose();
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = mController.getOutput(s_Swerve.getPose(), targetPose.get());
//        s_Swerve.runVelocity(
//            new ChassisSpeeds(
//                MathUtil.applyDeadband(speeds.vxMetersPerSecond, Deadbands.SWERVE_DEADBAND),
//                MathUtil.applyDeadband(speeds.vyMetersPerSecond, Deadbands.SWERVE_DEADBAND),
//                MathUtil.applyDeadband(speeds.omegaRadiansPerSecond, Deadbands.SWERVE_DEADBAND)));
//        ChassisSpeeds speeds = mController.calculate(s_Swerve.getPose(), targetPose.get(), 3.5, targetPose.get().getRotation());
        Logger.recordOutput("Align/ControllerOutputX", speeds.vxMetersPerSecond);
        Logger.recordOutput("Align/ControllerOutputY", speeds.vyMetersPerSecond);
        Logger.recordOutput("Align/ControllerOutputTheta", speeds.omegaRadiansPerSecond);
        s_Swerve.runVelocity(speeds);

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
    }

    @Override
    public boolean isFinished() {
        return debouncer.calculate(isAligned());
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.runVelocity(new ChassisSpeeds());
//        s_Swerve.stopWithX();
    }
}
