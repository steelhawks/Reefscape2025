package org.steelhawks.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;

import static org.steelhawks.util.PhoenixUtil.tryUntilOk;

public class ElevatorIOTalonFX implements ElevatorIO {

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final DigitalInput reverseLimit;

    private final MotionMagicVoltage motionMagicVoltage;
    private final VoltageOut voltageOut;
    private final DutyCycleOut dutyCycle;

    private final StatusSignal<Angle> leftPosition;
    private final StatusSignal<AngularVelocity> leftVelocity;
    private final StatusSignal<Voltage> leftVoltage;
    private final StatusSignal<Current> leftCurrent;
    private final StatusSignal<Temperature> leftTemp;

    private final StatusSignal<Angle> rightPosition;
    private final StatusSignal<AngularVelocity> rightVelocity;
    private final StatusSignal<Voltage> rightVoltage;
    private final StatusSignal<Current> rightCurrent;
    private final StatusSignal<Temperature> rightTemp;

    public ElevatorIOTalonFX() {
        leftMotor = new TalonFX(ElevatorConstants.LEFT_MOTOR_ID, Constants.getCANBus());
        rightMotor = new TalonFX(ElevatorConstants.RIGHT_MOTOR_ID, Constants.getCANBus());
        reverseLimit = new DigitalInput(0);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0 = new Slot0Configs()
            .withKP(ElevatorConstants.KP)
            .withKI(ElevatorConstants.KI)
            .withKD(ElevatorConstants.KD);
        config.Feedback.SensorToMechanismRatio = ElevatorConstants.REDUCTION;
        config.CurrentLimits.SupplyCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 1.5;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MAX_VELOCITY_ROT_PER_SEC;
        config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MAX_ACCELERATION_ROT_PER_SEC_2;
        tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config, 0.25));

        motionMagicVoltage = new MotionMagicVoltage(0.0);
        voltageOut = new VoltageOut(0.0);
        dutyCycle = new DutyCycleOut(0.0);

        leftPosition = leftMotor.getPosition();
        leftVelocity = leftMotor.getVelocity();
        leftVoltage = leftMotor.getSupplyVoltage();
        leftCurrent = leftMotor.getStatorCurrent();
        leftTemp = leftMotor.getDeviceTemp();

        rightPosition = rightMotor.getPosition();
        rightVelocity = rightMotor.getVelocity();
        rightVoltage = rightMotor.getSupplyVoltage();
        rightCurrent = rightMotor.getStatorCurrent();
        rightTemp = rightMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            leftPosition,
            leftVelocity,
            leftVoltage,
            leftCurrent,
            leftTemp,

            rightPosition,
            rightVelocity,
            rightVoltage,
            rightCurrent,
            rightTemp);
        ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor);
        zeroEncoders();
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
        inputs.leftTempCelsius = leftTemp.getValueAsDouble();
        Logger.recordOutput("Elevator/VelocityRot", leftVelocity.getValueAsDouble());

        inputs.rightConnected =
            BaseStatusSignal.refreshAll(
                rightPosition,
                rightVelocity,
                rightVoltage,
                rightCurrent,
                rightTemp).isOK();
        inputs.rightAppliedVolts = rightVoltage.getValueAsDouble();
        inputs.rightCurrentAmps = rightCurrent.getValueAsDouble();
        inputs.rightTempCelsius = rightTemp.getValueAsDouble();
    }

    @Override
    public void runElevator(double volts) {
        leftMotor.setControl(
            voltageOut.withOutput(volts)
                .withLimitForwardMotion(leftPosition.getValueAsDouble() > ElevatorConstants.MAX_ROTATIONS)
                .withLimitReverseMotion(!reverseLimit.get()));
    }

    @Override
    public void runElevatorViaSpeed(double speed) {
        leftMotor.setControl(
            dutyCycle.withOutput(speed)
                .withLimitForwardMotion(leftPosition.getValueAsDouble() > ElevatorConstants.MAX_ROTATIONS)
                .withLimitReverseMotion(!reverseLimit.get()));
    }

    @Override
    public void runPosition(double positionRot, double feedforward) {
        // you should use PositionVoltage instead because we already motion profile in Elevator.java
        // if you want to you could keep motion magic but then change positionRad to use inputs.goal by caching it in an object and using that.
        // at this state this would most likely not work
        leftMotor.setControl(
            motionMagicVoltage
                .withPosition(positionRot)
                .withFeedForward(feedforward)
                .withLimitForwardMotion(leftPosition.getValueAsDouble() > ElevatorConstants.MAX_ROTATIONS)
                .withLimitReverseMotion(!reverseLimit.get()));
    }

    @Override
    public void zeroEncoders() {
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
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
        rightMotor.stopMotor();
    }
}
