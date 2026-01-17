package org.steelhawks.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import org.steelhawks.Constants;

import static org.steelhawks.util.PhoenixUtil.tryUntilOk;

public class ElevatorIOTalonFX implements ElevatorIO {

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private final PositionTorqueCurrentFOC positionTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrent;
    private final VoltageOut voltageOut;
    private final DutyCycleOut dutyCycle;

    private final StatusSignal<Angle> leftPosition;
    private final StatusSignal<AngularVelocity> leftVelocity;
    private final StatusSignal<Voltage> leftVoltage;
    private final StatusSignal<Current> leftCurrent;
    private final StatusSignal<Current> leftTorqueCurrent;
    private final StatusSignal<Temperature> leftTemp;

    private final StatusSignal<Angle> rightPosition;
    private final StatusSignal<AngularVelocity> rightVelocity;
    private final StatusSignal<Voltage> rightVoltage;
    private final StatusSignal<Current> rightCurrent;
    private final StatusSignal<Current> rightTorqueCurrent;
    private final StatusSignal<Temperature> rightTemp;

    public ElevatorIOTalonFX() {
        leftMotor = new TalonFX(ElevatorConstants.LEFT_MOTOR_ID, Constants.getCANBus());
        rightMotor = new TalonFX(ElevatorConstants.RIGHT_MOTOR_ID, Constants.getCANBus());
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0 = new Slot0Configs()
            .withKP(ElevatorConstants.KP.get())
            .withKI(ElevatorConstants.KI)
            .withKD(ElevatorConstants.KD.get());
        config.Feedback.SensorToMechanismRatio = ElevatorConstants.REDUCTION;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config, 0.25));

        positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0.0)
            .withSlot(0)
            .withUpdateFreqHz(0.0);
        torqueCurrent = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
        voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
        dutyCycle = new DutyCycleOut(0.0).withUpdateFreqHz(0.0);

        leftPosition = leftMotor.getPosition();
        leftVelocity = leftMotor.getVelocity();
        leftVoltage = leftMotor.getSupplyVoltage();
        leftCurrent = leftMotor.getStatorCurrent();
        leftTorqueCurrent = leftMotor.getTorqueCurrent();
        leftTemp = leftMotor.getDeviceTemp();

        rightPosition = rightMotor.getPosition();
        rightVelocity = rightMotor.getVelocity();
        rightVoltage = rightMotor.getSupplyVoltage();
        rightCurrent = rightMotor.getStatorCurrent();
        rightTorqueCurrent = rightMotor.getTorqueCurrent();
        rightTemp = rightMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            leftPosition,
            leftVelocity,
            leftVoltage,
            leftCurrent,
            leftTorqueCurrent,
            leftTemp,

            rightPosition,
            rightVelocity,
            rightVoltage,
            rightCurrent,
            rightTorqueCurrent,
            rightTemp);
        ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leftConnected =
            BaseStatusSignal.refreshAll(
                leftPosition,
                leftVelocity,
                leftVoltage,
                leftCurrent,
                leftTemp).isOK();
        inputs.positionRot = leftPosition.getValueAsDouble();
        inputs.velocityRotPerSec = leftVelocity.getValueAsDouble();
        inputs.leftAppliedVolts = leftVoltage.getValueAsDouble();
        inputs.leftCurrentAmps = leftCurrent.getValueAsDouble();
        inputs.leftTorqueCurrentAmps = leftTorqueCurrent.getValueAsDouble();
        inputs.leftTempCelsius = leftTemp.getValueAsDouble();

        inputs.rightConnected =
            BaseStatusSignal.refreshAll(
                rightPosition,
                rightVelocity,
                rightVoltage,
                rightCurrent,
                rightTemp).isOK();
        inputs.rightAppliedVolts = rightVoltage.getValueAsDouble();
        inputs.rightCurrentAmps = rightCurrent.getValueAsDouble();
        inputs.rightTorqueCurrentAmps = rightTorqueCurrent.getValueAsDouble();
        inputs.rightTempCelsius = rightTemp.getValueAsDouble();
    }

    @Override
    public void runElevator(double volts) {
        leftMotor.setControl(
            voltageOut.withOutput(volts));
    }

    /**
     * Current output
     * @param output Current in amperes
     */
    @Override
    public void runOpenLoop(double output) {
        leftMotor.setControl(
            torqueCurrent.withOutput(output));
    }

    @Override
    public void runElevatorViaSpeed(double speed) {
        leftMotor.setControl(
            dutyCycle.withOutput(speed));
    }

    @Override
    public void runPosition(double positionRot, double feedforward) {
        leftMotor.setControl(
            positionTorqueCurrentFOC
                .withPosition(positionRot)
                .withFeedForward(feedforward));
    }

    @Override
    public void zeroEncoders() {
        new Thread(() -> {
            leftMotor.setPosition(0);
            rightMotor.setPosition(0);
        }).start();
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        new Thread(
            () -> leftMotor.setNeutralMode(
                enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast)).start();
    }

    @Override
    public void stop() {
        leftMotor.stopMotor();
    }
}
