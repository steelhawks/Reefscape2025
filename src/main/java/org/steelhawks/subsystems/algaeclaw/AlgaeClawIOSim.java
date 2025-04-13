package org.steelhawks.subsystems.algaeclaw;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import org.steelhawks.Constants;
import org.steelhawks.subsystems.elevator.ElevatorConstants;

public class AlgaeClawIOSim implements AlgaeClawIO {

//    private final MechanismRoot2d armPivot;
    private final Mechanism2d canvas;
    private double width, height;

    private final SingleJointedArmSim sim;

    private double pivotVoltage = 0.0;
    private double spinVoltage = 0.0;
    private double spinPosition = 0.0;

    public AlgaeClawIOSim() {
        width = AlgaeClawConstants.ARM_LENGTH * 2 + Units.inchesToMeters(3);
        height = ElevatorConstants.MAX_RADIANS + AlgaeClawConstants.ARM_LENGTH + Units.inchesToMeters(3);
        canvas = new Mechanism2d(width, height);
//        armPivot = canvas.getRoot("Arm Pivot", width / 2.0, Constants.Elevator.MIN_HEIGHT_METERS - Constants.Arm.DISTANCE_FROM_PIVOT_TO_TOP_OF_ELEVATOR);
        sim = new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            AlgaeClawConstants.GEAR_RATIO,
            AlgaeClawConstants.MOMENT_OF_INERTIA,
            AlgaeClawConstants.ARM_LENGTH,
            AlgaeClawConstants.MIN_PIVOT_RADIANS,
            AlgaeClawConstants.MAX_PIVOT_RADIANS,
            false,
            AlgaeClawConstants.MIN_PIVOT_RADIANS);
    }

    @Override
    public void updateInputs(AlgaeClawIOInputs inputs) {
        inputs.pivotConnected = true;
        inputs.pivotPosition = sim.getAngleRads();
        inputs.pivotVelocity = sim.getVelocityRadPerSec();
        inputs.pivotAppliedVolts = pivotVoltage;
        inputs.pivotCurrent = sim.getCurrentDrawAmps();
        inputs.pivotTemperature = sim.getCurrentDrawAmps() * 0.1; // Simulated temperature

        inputs.spinConnected = true;
        inputs.spinPosition = spinPosition;
        inputs.spinVelocity = spinVelocity();
        inputs.spinAppliedVolts = spinVoltage;
//        inputs.spinCurrent = Math.signum(spinVoltage) * AlgaeClawConstants.CURRENT_THRESHOLD_TO_HAVE_ALGAE; // Simulated current
        inputs.spinCurrent = spinVoltage > 0 ? AlgaeClawConstants.CURRENT_THRESHOLD_TO_HAVE_ALGAE : 0; // Simulated current
        inputs.spinTemperature = inputs.spinCurrent * 0.1; // Simulated temperature

        inputs.encoderConnected = true;
        inputs.encoderPosition = sim.getAngleRads();
        inputs.encoderVelocity = sim.getVelocityRadPerSec();
        inputs.encoderAppliedVolts = pivotVoltage * 0.01;

        sim.update(Constants.UPDATE_LOOP_DT);
    }

    private double spinVelocity() {
        Timer timer = new Timer();
        double prevPosition = spinPosition;
        timer.start();
        double spinVelocity = (spinPosition - prevPosition) / timer.get();
        timer.stop();
        timer.reset();
        return spinVelocity;
    }

    @Override
    public void runSpin(double speed) {
        spinVoltage = speed * 12.0;
        spinPosition += speed * Constants.UPDATE_LOOP_DT;
    }

    @Override
    public void stopSpin() {
        spinVoltage = 0.0;
    }

    @Override
    public void runPivot(double volts) {
        pivotVoltage = volts;
        sim.setInputVoltage(volts);
    }

    @Override
    public void runPivotViaSpeed(double speed) {
        pivotVoltage = speed * 12.0;
        sim.setInputVoltage(pivotVoltage);
    }

    @Override
    public void stopPivot() {
        sim.setInputVoltage(0.0);
    }
}
