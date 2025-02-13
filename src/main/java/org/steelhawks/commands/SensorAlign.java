package org.steelhawks.commands;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.RobotContainer;
import org.steelhawks.Reefstate;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.VirtualSubsystem;


public class SensorAlign extends VirtualSubsystem {

    private static final Swerve s_Swerve = RobotContainer.s_Swerve;

    private static final double LEFT_TOLERANCE = 0.005;
    private static final double LEFT_KP = 0;
    private static final double LEFT_KI = 0;
    private static final double LEFT_KD = 0;

    private static final double RIGHT_TOLERANCE = 0.005;
    private static final double RIGHT_KP = 0;
    private static final double RIGHT_KI = 0;
    private static final double RIGHT_KD = 0;

    private static final double DIST_TOLERANCE = 0.005;
    private static final double DIST_KP = 0;
    private static final double DIST_KI = 0;
    private static final double DIST_KD = 0;

    private static final int LEFT_ID = 19;
    private static final int RIGHT_ID = 20;

//    private static final double LEFT_SENSOR_ANGLE = 31; // degrees
    private static final double TARGET_DISTANCE = Units.inchesToMeters(2.0);
    private static final double LEFT_ALIGN_THRESHOLD = Units.inchesToMeters(5);
    private static final double RIGHT_ALIGN_THRESHOLD = Units.inchesToMeters(8);

    // left encoder measured distance when aligned to the left coral branch on the reef: 0.375 m
    // right encoder measured distance when aligned to the left coral branch on the reef: 0.365 m
    private final CANrange mLeftCANrange;
    private final CANrange mRightCANrange;

    private final PIDController mLeftController;
    private final PIDController mRightController;
    private final PIDController mDistanceController;
    private final Debouncer mDebouncer;

    private final Alert leftDisconnected;
    private final Alert rightDisconnected;

    private final StatusSignal<Distance> mLeftDist;
    private final StatusSignal<Distance> mRightDist;

    public SensorAlign() {
        mLeftCANrange = new CANrange(LEFT_ID, Constants.getCANBus());
        mRightCANrange = new CANrange(RIGHT_ID, Constants.getCANBus());

        mLeftController = new PIDController(LEFT_KP, LEFT_KI, LEFT_KD);
        mLeftController.setTolerance(LEFT_TOLERANCE);

        mRightController = new PIDController(RIGHT_KP, RIGHT_KI, RIGHT_KD);
        mRightController.setTolerance(RIGHT_TOLERANCE);

        mDistanceController = new PIDController(DIST_KP, DIST_KI, DIST_KD);
        mDistanceController.setTolerance(DIST_TOLERANCE);

        mDebouncer = new Debouncer(.5, DebounceType.kBoth);

        leftDisconnected = new Alert(
            "Left CANrange is disconnected", AlertType.kError);
        rightDisconnected = new Alert(
            "Left CANrange is disconnected", AlertType.kError);

        mLeftDist = mLeftCANrange.getDistance();
        mRightDist = mRightCANrange.getDistance();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            mLeftDist,
            mRightDist);
        ParentDevice.optimizeBusUtilizationForAll(mLeftCANrange, mRightCANrange);
    }

    @Override
    public void periodic() {
        boolean leftConnected = BaseStatusSignal.refreshAll(mLeftDist).isOK();
        boolean rightConnected = BaseStatusSignal.refreshAll(mRightDist).isOK();

        Logger.recordOutput("Align/LeftConnected", leftConnected);
        Logger.recordOutput("Align/LeftDistance", mLeftDist.getValueAsDouble());

        Logger.recordOutput("Align/RightConnected", rightConnected);
        Logger.recordOutput("Align/RightDistance", mRightDist.getValueAsDouble());

        leftDisconnected.set(!leftConnected);
        rightDisconnected.set(!rightConnected);
    }

    public Command alignParallelToNearestReefCommand() {
        Pose2d closestReefSectionPose = Reefstate.getClosestReefSectionPose();

        return DriveCommands.joystickDriveAtAngle(() -> 0, () -> 0, () -> closestReefSectionPose.getRotation());
    }

    public Command forwardUntil() {
        return Commands.run(
            () -> {
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        mDistanceController.calculate(mLeftDist.getValueAsDouble(), TARGET_DISTANCE),
                        0.0,
                        0.0);
                s_Swerve.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        AllianceFlip.shouldFlip()
                            ? s_Swerve.getRotation().plus(new Rotation2d(Math.PI))
                            : s_Swerve.getRotation()));
            }, s_Swerve)
            .until(() -> mDebouncer.calculate(mDistanceController.atSetpoint()));
    }

    public Command alignParallelToReefCommand(Pose2d reefPose) {
        Rotation2d reefRotation = reefPose.getRotation();
        return DriveCommands.joystickDriveAtAngle(() -> 0, () -> 0, () -> new Rotation2d(reefRotation.getRadians() + Math.PI));
    }

    public Command alignLeft() {
        return Commands.run(
            () -> {
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        0.0,
                        mLeftController.calculate(mLeftDist.getValueAsDouble(), LEFT_ALIGN_THRESHOLD),
                        0.0);
                s_Swerve.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds,
                    AllianceFlip.shouldFlip()
                        ? s_Swerve.getRotation().plus(new Rotation2d(Math.PI))
                        : s_Swerve.getRotation()));
            }, s_Swerve)
            .until(() -> mDebouncer.calculate(mLeftController.atSetpoint()));
    }

    public Command alignRight() {
        return Commands.run(
            () -> {
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        0.0,
                        mRightController.calculate(mRightDist.getValueAsDouble(), RIGHT_ALIGN_THRESHOLD),
                        0.0);
                s_Swerve.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        AllianceFlip.shouldFlip()
                            ? s_Swerve.getRotation().plus(new Rotation2d(Math.PI))
                            : s_Swerve.getRotation()));
            }, s_Swerve)
            .until(() -> mDebouncer.calculate(mRightController.atSetpoint()));
    }
}
