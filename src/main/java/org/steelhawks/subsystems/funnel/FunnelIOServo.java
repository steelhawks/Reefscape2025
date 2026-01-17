package org.steelhawks.subsystems.funnel;

import edu.wpi.first.wpilibj.Servo;

public class FunnelIOServo implements FunnelIO {

	private final Servo funnelServo;

	public FunnelIOServo() {
		funnelServo = new Servo(FunnelConstants.SERVO_CHANNEL);
	}

	@Override
	public void updateInputs(FunnelIOInputs inputs) {
		inputs.positionDeg = funnelServo.getAngle();
	}

	@Override
	public void runServo(double degrees) {
		funnelServo.setAngle(degrees);
	}
}
