package org.steelhawks.subsystems.claw;

import com.ctre.phoenix6.CANBus;
import org.steelhawks.Constants;
import org.steelhawks.util.ConstantsFactory;

public class ClawConstants {

    public static final CANBus CLAW_CANBUS = new CANBus("");
    public static final int CAN_RANGE_ID_OMEGA = 17;

    public static final Integer CLAW_INTAKE_MOTOR_ID = ConstantsFactory.value(16, 15);

    public static final Double CLAW_INTAKE_GEAR_RATIO = ConstantsFactory.value(1.0, 2.0 / 1.0);

    public static final Double CLAW_SHOOT_SPEED = ConstantsFactory.value(0.1, 0.15);
    public static final Double CLAW_SECONDARY_SHOOT_SPEED = ConstantsFactory.value(0.135, 0.125);
    public static final Double CLAW_INTAKE_SPEED = ConstantsFactory.value(0.1, 0.1);
    public static Double CLAW_MOTOR_MAX_RPM = 6784.0;
}
