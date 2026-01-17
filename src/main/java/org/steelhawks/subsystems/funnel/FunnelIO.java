package org.steelhawks.subsystems.funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {

	@AutoLog
	class FunnelIOInputs {
		public double positionDeg = 0.0;
	}

	default void updateInputs(FunnelIOInputs inputs) {}

	default void runServo(double degrees) {}
}
