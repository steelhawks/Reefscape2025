package org.steelhawks.commands;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import org.steelhawks.util.VirtualSubsystem;


public class SensorAlign extends VirtualSubsystem {

    private static final Swerve s_Swerve = RobotContainer.s_Swerve;

    private static final double TOLERANCE = 0.005;
    private static final double KP = 0;
    private static final double KI = 0;
    private static final double KD = 0;
    private static final int LEFT_ID = 19;
    private static final int RIGHT_ID = 20;

//    private static final double LEFT_SENSOR_ANGLE = 31; // degrees
    private static final double TARGET_DISTANCE = Units.inchesToMeters(2.0);
    private static final double ALIGN_THRESHOLD = Units.inchesToMeters(5);

    private final CANrange mLeftCANrange;
    private final CANrange mRightCANrange;
    private final PIDController mLeftController;

    private final Alert leftDisconnected;
    private final Alert rightDisconnected;

    private final StatusSignal<Distance> mLeftDist;
    private final StatusSignal<Distance> mRightDist;

    public SensorAlign() {
        mLeftCANrange = new CANrange(LEFT_ID, Constants.getCANBus());
        mRightCANrange = new CANrange(RIGHT_ID, Constants.getCANBus());
        mLeftController = new PIDController(KP, KI, KD);
        mLeftController.setTolerance(TOLERANCE);

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

    public Command alignParallelToReefCommand() {
        Pose2d closestReefSectionPose = Reefstate.getClosestReefSectionPose();
        Rotation2d reefRotation = closestReefSectionPose.getRotation();

        return DriveCommands.joystickDriveAtAngle(() -> 0, () -> 0, () -> reefRotation);
}


    public Command alignToLeftCoralCommand() {
        return Commands.run(
            () -> {
               mLeftController.setSetpoint(TARGET_DISTANCE);
               
            }, s_Swerve);
    }
}
