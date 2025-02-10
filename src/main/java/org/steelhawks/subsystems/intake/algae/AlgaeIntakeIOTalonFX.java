package org.steelhawks.subsystems.intake.algae;

import org.steelhawks.Constants;
import org.steelhawks.Constants.RobotType;
import org.steelhawks.subsystems.intake.IntakeConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class AlgaeIntakeIOTalonFX implements AlgaeIntakeIO {

    private final IntakeConstants constants;

    private final TalonFX mIntakeMotor;
    private final TalonFX mPivotMotor;
    private final CANcoder mCANcoder;

    private final DigitalInput mLimitSwitch;

    private final StatusSignal<Angle> intakePosition;
    private final StatusSignal<AngularVelocity> intakeVelocity;
    private final StatusSignal<Voltage> intakeVoltage;
    private final StatusSignal<Current> intakeCurrent;
    private final StatusSignal<Temperature> intakeTemp;

    private final StatusSignal<Angle> pivotPosition;
    private final StatusSignal<AngularVelocity> pivotVelocity;
    private final StatusSignal<Voltage> pivotVoltage;
    private final StatusSignal<Current> pivotCurrent;
    private final StatusSignal<Temperature> pivotTemp;

    private final StatusSignal<Boolean> magnetFault;
    private final StatusSignal<Angle> canCoderPosition;
    private final StatusSignal<AngularVelocity> canCoderVelocity;

    public AlgaeIntakeIOTalonFX() {
        switch (Constants.getRobot()) {
            case ALPHABOT -> constants = IntakeConstants.ALPHA;
            case HAWKRIDER -> constants = IntakeConstants.HAWKRIDER;
            default -> constants = IntakeConstants.OMEGA;
        }

        mIntakeMotor = new TalonFX(constants.ALGAE_INTAKE_MOTOR_ID, Constants.getCANBus());
        mPivotMotor = new TalonFX(constants.ALGAE_PIVOT_MOTOR_ID, Constants.getCANBus());
        mCANcoder = new CANcoder(constants.ALGAE_CANCODER_ID, Constants.getCANBus());
        mLimitSwitch = new DigitalInput(constants.ALGAE_LIMIT_SWITCH_ID);

        var intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        mIntakeMotor.getConfigurator().apply(intakeConfig);
        mPivotMotor.getConfigurator().apply(pivotConfig);

        mCANcoder.getConfigurator().apply(
            new CANcoderConfiguration().withMagnetSensor(
                new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)));

        mCANcoder.setPosition(0);

        intakePosition = mIntakeMotor.getPosition();
        intakeVelocity = mIntakeMotor.getVelocity();
        intakeVoltage = mIntakeMotor.getSupplyVoltage();
        intakeCurrent = mIntakeMotor.getStatorCurrent();
        intakeTemp = mIntakeMotor.getDeviceTemp();

        pivotPosition = mPivotMotor.getPosition();
        pivotVelocity = mPivotMotor.getVelocity();
        pivotVoltage = mPivotMotor.getSupplyVoltage();
        pivotCurrent = mPivotMotor.getStatorCurrent();
        pivotTemp = mPivotMotor.getDeviceTemp();

        magnetFault = mCANcoder.getFault_BadMagnet();
        canCoderPosition = mCANcoder.getPosition();
        canCoderVelocity = mCANcoder.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            intakePosition,
            intakeVelocity,
            intakeVoltage,
            intakeCurrent,
            intakeTemp,

            pivotPosition,
            pivotVelocity,
            pivotVoltage,
            pivotCurrent,
            pivotTemp,

            magnetFault,
            canCoderPosition,
            canCoderVelocity);

        ParentDevice.optimizeBusUtilizationForAll(mIntakeMotor, mPivotMotor, mCANcoder);
    }

    boolean hitLimit;

    @Override
    public void updateInputs(AlgaeIntakeIOInputs inputs) {
        inputs.intakeConnected =
            BaseStatusSignal.refreshAll(
                intakePosition,
                intakeVelocity,
                intakeVoltage,
                intakeCurrent,
                intakeTemp).isOK();
        inputs.intakePositionRad = Units.rotationsToRadians(intakePosition.getValueAsDouble());
        inputs.intakeVelocityRadPerSec = Units.rotationsToRadians(intakeVelocity.getValueAsDouble());
        inputs.intakeAppliedVolts = intakeVoltage.getValueAsDouble();
        inputs.intakeCurrentAmps = intakeCurrent.getValueAsDouble();
        inputs.intakeTempCelsius = intakeTemp.getValueAsDouble();

        inputs.pivotConnected =
            BaseStatusSignal.refreshAll(
                pivotPosition,
                pivotVelocity,
                pivotVoltage,
                pivotCurrent,
                pivotTemp).isOK();
        inputs.pivotPositionRad = Units.rotationsToRadians(pivotPosition.getValueAsDouble());
        inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotVelocity.getValueAsDouble());
        inputs.pivotAppliedVolts = pivotVoltage.getValueAsDouble();
        inputs.pivotCurrentAmps = pivotCurrent.getValueAsDouble();
        inputs.pivotTempCelsius = pivotTemp.getValueAsDouble();

        inputs.encoderConnected =
            BaseStatusSignal.refreshAll(
                magnetFault,
                canCoderPosition,
                canCoderVelocity).isOK();
        inputs.magnetGood = !magnetFault.getValue();
        inputs.encoderPositionRad = canCoderPosition.getValueAsDouble();
        inputs.encoderVelocityRadPerSec = canCoderVelocity.getValueAsDouble();

        inputs.limitSwitchConnected = mLimitSwitch.getChannel() == constants.ALGAE_LIMIT_SWITCH_ID;
        inputs.limitSwitchPressed = !mLimitSwitch.get();

        if (Constants.getRobot() == RobotType.ALPHABOT) {
            // TODO: ACCOUNT FOR GEAR RATIO IN ALGAE INTAKE

            // if (Constants.getRobot() == RobotType.ALPHABOT) {
            //     leftPos *= ELEVATOR_GEAR_RATIO;
            //     rightPos *= ELEVATOR_GEAR_RATIO;
            //     leftVelo *= ELEVATOR_GEAR_RATIO;
            //     rightVelo *= ELEVATOR_GEAR_RATIO;
            // }
    



            inputs.encoderConnected = inputs.pivotConnected;
            inputs.magnetGood = inputs.pivotConnected;
            inputs.encoderPositionRad = inputs.pivotPositionRad;
            inputs.encoderVelocityRadPerSec = inputs.pivotVelocityRadPerSec;
        }

        hitLimit = inputs.limitSwitchPressed;
    }

    @Override
    public void runPivot(double volts) {
        if (hitLimit) {
            stopPivot();
            return;
        }

        mPivotMotor.setVoltage(volts);
    }

    @Override
    public void runPivotManual(double speed) {
        if (hitLimit && speed > 0) {
            stopPivot();
            return;
        }

        mPivotMotor.set(speed);
    }

    @Override
    public void stopPivot() {
        mPivotMotor.stopMotor();
    }

    @Override
    public void runIntake(double speed) {
        mIntakeMotor.set(speed);
    }

    @Override
    public void stopIntake() {
        mIntakeMotor.stopMotor();
    }
}
