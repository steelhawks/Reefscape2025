package org.steelhawks.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.steelhawks.Constants;
import org.steelhawks.util.Conversions;

public class ElevatorIOSim implements ElevatorIO {

    private static final double ELEVATOR_WEIGHT = 40; // kg
    private static final double SPROCKET_RAD = // the driving drum
        Units.inchesToMeters(1.888);
    private static final double ELEVATOR_GEARING = 10.0 / 1.0;
    private static final double MIN_HEIGHT = 0; //m
    public static final double MAX_HEIGHT =
        Conversions.rotationsToMeters(
            Units.radiansToRotations(ElevatorConstants.MAX_RADIANS), 2 * Math.PI * SPROCKET_RAD); //m
    private static final double ELEVATOR_WIDTH =
        Units.inchesToMeters(27);

    private final PIDController mController;
    private final ElevatorVisualizer mVisualizer;
    private final ElevatorSim mElevatorSim;
    private final DCMotor mMotor;

    private boolean runningProfile = false;
    private double elevatorPosition = 0.0;
    private double appliedVolts = 0.0;

    public ElevatorIOSim() {
        mController =
            new PIDController(
                ElevatorConstants.KP,
                ElevatorConstants.KI,
                ElevatorConstants.KD);
        mMotor = DCMotor.getFalcon500(2);
        mElevatorSim =
            new ElevatorSim(
                LinearSystemId.createElevatorSystem(
                    mMotor,
                    ELEVATOR_WEIGHT,
                    SPROCKET_RAD,
                    ELEVATOR_GEARING),
                mMotor,
                MIN_HEIGHT,
                MAX_HEIGHT,
                true,
                0);
        mVisualizer =
            new ElevatorVisualizer(
                mElevatorSim::getPositionMeters,
                ELEVATOR_WIDTH,
                MAX_HEIGHT);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        mElevatorSim.update(Constants.UPDATE_LOOP_DT);
        runningProfile = inputs.shouldRunProfile;
        if (!runningProfile) {
            mController.setSetpoint(inputs.leftPositionRad);
        }

        inputs.leftConnected = true;
        inputs.leftPositionRad =
            Units.rotationsToRadians(
                Conversions.metersToRotations(mElevatorSim.getPositionMeters(), 2 * Math.PI * SPROCKET_RAD));
        inputs.leftVelocityRadPerSec =
            Units.rotationsToRadians(
                Conversions.metersToRotations(mElevatorSim.getVelocityMetersPerSecond(), 2 * Math.PI * SPROCKET_RAD));
        inputs.leftAppliedVolts = appliedVolts;
        inputs.leftCurrentAmps = mElevatorSim.getCurrentDrawAmps();
        elevatorPosition = inputs.leftPositionRad;

        inputs.rightConnected = true;
        inputs.rightPositionRad =
            inputs.leftPositionRad;
        inputs.rightVelocityRadPerSec =
            inputs.leftVelocityRadPerSec;
        inputs.rightAppliedVolts = appliedVolts;
        inputs.rightCurrentAmps = inputs.leftCurrentAmps;

        inputs.limitSwitchConnected = true;
        inputs.limitSwitchPressed = mElevatorSim.hasHitLowerLimit();
        inputs.atTopLimit = mElevatorSim.hasHitUpperLimit();

        mVisualizer.update();
    }

    @Override
    public void runElevator(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12, 12);
        mElevatorSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
    }

    @Override
    public void runElevatorViaSpeed(double speed) {
        double convertToVolts = speed * 12;
        mElevatorSim.setInputVoltage(MathUtil.clamp(convertToVolts, -12, 12));
    }

    @Override
    public void runPosition(double positionRad, double feedforward) {
        double fb = mController.calculate(elevatorPosition, positionRad);
        if (runningProfile) {
            double volts = MathUtil.clamp(fb + feedforward, -12, 12);
            appliedVolts = volts;
            mElevatorSim.setInputVoltage(volts);
        }
    }

    @Override
    public void stop() {
        appliedVolts = 0;
        mElevatorSim.setInputVoltage(0);
    }
}
