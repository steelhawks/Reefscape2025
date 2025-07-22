package org.steelhawks.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

public class SwerveDriveController {

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController thetaController;
    private boolean firstRun = true;

    public SwerveDriveController(
        ProfiledPIDController xController, ProfiledPIDController yController, ProfiledPIDController thetaController) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveDriveController withLinearTolerance(double xyTolerance) {
        xController.setTolerance(xyTolerance);
        yController.setTolerance(xyTolerance);
        return this;
    }

    public SwerveDriveController withRotationalTolerance(double thetaTolerance) {
        thetaController.setTolerance(thetaTolerance);
        return this;
    }

    public ProfiledPIDController getXController() {
        return xController;
    }

    public ProfiledPIDController getYController() {
        return yController;
    }

    public ProfiledPIDController getThetaController() {
        return thetaController;
    }

    public void reset(Pose2d measurement) {
        thetaController.reset(measurement.getRotation().getRadians());
        xController.reset(measurement.getTranslation().getX());
        yController.reset(measurement.getTranslation().getY());
    }

    public ChassisSpeeds getOutput(Pose2d measurement, Pose2d setpoint) {
        if (firstRun) {
            reset(measurement);
            firstRun = false;
        }

        double xOutput = xController.calculate(measurement.getX(), setpoint.getX());
        double yOutput = yController.calculate(measurement.getY(), setpoint.getY());
        double thetaOutput = thetaController.calculate(measurement.getRotation().getRadians(), setpoint.getRotation().getRadians());

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            xOutput,
            yOutput,
            thetaOutput,
            measurement.getRotation());
    }

    public ChassisSpeeds getError() {
        return new ChassisSpeeds(
            xController.getVelocityError(),
            yController.getVelocityError(),
            thetaController.getVelocityError());
    }
}
