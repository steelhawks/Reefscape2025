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
import org.steelhawks.subsystems.LED;
import org.steelhawks.subsystems.LED.LEDColor;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.VirtualSubsystem;


public class SensorAlign extends VirtualSubsystem {

    private static final Swerve s_Swerve = RobotContainer.s_Swerve;

    private static final double LEFT_TOLERANCE = 0.001;
    private static final double LEFT_KP = 0.8;
    private static final double LEFT_KI = 0;
    private static final double LEFT_KD = 0;

    private static final double RIGHT_TOLERANCE = 0.005;
    private static final double RIGHT_KP = 0;
    private static final double RIGHT_KI = 0;
    private static final double RIGHT_KD = 0;

    private static final double DIST_TOLERANCE = Units.inchesToMeters(0.005); // 0.00013 meters
    private static final double DIST_KP = 0.9;
    private static final double DIST_KI = 0;
    private static final double DIST_KD = 0;

    private static final int LEFT_ID = 19;
    private static final int RIGHT_ID = 20;

//    private static final double LEFT_SENSOR_ANGLE = 31; // degrees
    private static final double TARGET_DISTANCE = Units.inchesToMeters(3.0); // 0.051 meters
    private static final double LEFT_ALIGN_THRESHOLD = 0.39;
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

    public Command forwardUntil(Rotation2d angle) {
        return Commands.run(
            () -> {
                double output = mDistanceController.calculate(mLeftDist.getValueAsDouble(), TARGET_DISTANCE);
                double alignOutput = s_Swerve.getAlign().calculate(s_Swerve.getRotation().getRadians(), angle.getRadians());
                Logger.recordOutput("Align/Feedback", output);
                Logger.recordOutput("Align/AngleFeedback", alignOutput);
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        -output,
                        0.0,
                        alignOutput);
                s_Swerve.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        AllianceFlip.shouldFlip()
                            ? s_Swerve.getRotation().plus(new Rotation2d(Math.PI))
                            : s_Swerve.getRotation()));
            }, s_Swerve)
            .until(() -> mDebouncer.calculate(mDistanceController.atSetpoint()))
            .finallyDo(() -> LED.getInstance().flashCommand(LEDColor.GREEN, .2, 2));
    }

    public Command alignLeft(Rotation2d angle) {
        return Commands.run(
            () -> {
                double alignOutput = s_Swerve.getAlign().calculate(s_Swerve.getRotation().getRadians(), angle.getRadians());
                double output = mLeftController.calculate(mLeftDist.getValueAsDouble(), LEFT_ALIGN_THRESHOLD);
                Logger.recordOutput("Align/AngleFeedback", alignOutput);
                Logger.recordOutput("Align/LeftFeedback", output);
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        0.0,
                        output,
                        alignOutput);
                s_Swerve.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds,
                    AllianceFlip.shouldFlip()
                        ? s_Swerve.getRotation().plus(new Rotation2d(Math.PI))
                        : s_Swerve.getRotation()));
            }, s_Swerve)
            .until(() -> mDebouncer.calculate(mLeftController.atSetpoint()))
            .finallyDo(() -> LED.getInstance().flashCommand(LEDColor.GREEN, .2, 2));
    }

    public Command alignRight(Rotation2d angle) {
        return Commands.run(
            () -> {
                double alignOutput = s_Swerve.getAlign().calculate(s_Swerve.getRotation().getRadians(), angle.getRadians());
                double output = mRightController.calculate(mRightDist.getValueAsDouble());
                Logger.recordOutput("Align/AngleFeedback", alignOutput);
                Logger.recordOutput("Align/RightFeedback", output);
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        0.0,
                        -output,
                        0.0);
                s_Swerve.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        AllianceFlip.shouldFlip()
                            ? s_Swerve.getRotation().plus(new Rotation2d(Math.PI))
                            : s_Swerve.getRotation()));
            }, s_Swerve)
            .until(() -> mDebouncer.calculate(mRightController.atSetpoint()))
            .finallyDo(() -> LED.getInstance().flashCommand(LEDColor.GREEN, .2, 2));
    }
}
