package org.steelhawks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.util.Conversions;

public class RobotVisualizer {

    private static RobotVisualizer INSTANCE;

    public static RobotVisualizer getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotVisualizer();
        }
        return INSTANCE;
    }

    /* ------------- Elevator ------------- */

    private static final double LENGTH_BETWEEN_STAGES = Units.inchesToMeters(18.0);
    private static final double MAXIMUM_EXTENSION = Conversions.rotationsToMeters(
        Units.radiansToRotations(ElevatorConstants.MAX_RADIANS), Math.PI * 2 * Units.inchesToMeters(1.888));
    private static final double STAGE_WIDTH = Units.inchesToMeters(18.0);

    private static final double STAGE0_HEIGHT = Units.inchesToMeters(28.0);
    private static final double STAGE1_HEIGHT = Units.inchesToMeters(29.963);
    private static final double STAGE2_HEIGHT = Units.inchesToMeters(29.991);
    private static final double STAGE2_Y = Units.inchesToMeters(2.044);

    private static final double STAGE3_HEIGHT = Units.inchesToMeters(5.910);
    private static final double STAGE3_Y = Units.inchesToMeters(3.044);
    private static final double DISTANCE_BETWEEN_STAGES = Units.inchesToMeters(0.50);

    private static final double WIDTH = LENGTH_BETWEEN_STAGES + Units.inchesToMeters(32.0);
    private static final double HEIGHT = MAXIMUM_EXTENSION;

    private final Mechanism2d canvas;
    private final MechanismRoot2d elevatorLeftStage0;
    private final MechanismRoot2d elevatorRightStage0;
    private final MechanismRoot2d elevatorLeftStage1;
    private final MechanismRoot2d elevatorRightStage1;
    private final MechanismRoot2d elevatorLeftStage2;
    private final MechanismRoot2d elevatorRightStage2;
    private final MechanismRoot2d elevatorLeftStage3;
    private final MechanismRoot2d elevatorRightStage3;

    /* ------------- AlgaeClaw ------------- */



    private RobotVisualizer() {
        canvas = new Mechanism2d(WIDTH, HEIGHT);
        elevatorLeftStage0 = canvas.getRoot("ElevatorLeftStage0", ((WIDTH - LENGTH_BETWEEN_STAGES) / 2.0), 0.0);
        elevatorRightStage0 = canvas.getRoot("ElevatorRightStage0", ((WIDTH + LENGTH_BETWEEN_STAGES) / 2.0), 0.0);

        elevatorLeftStage0.append(new MechanismLigament2d("Stage0Left", STAGE0_HEIGHT, 90, STAGE_WIDTH, new Color8Bit(Color.kRed)));
        elevatorRightStage0.append(new MechanismLigament2d("Stage0Right", STAGE0_HEIGHT, 90, STAGE_WIDTH, new Color8Bit(Color.kRed)));

        elevatorLeftStage1 = canvas.getRoot("ElevatorLeftStage1", ((WIDTH - LENGTH_BETWEEN_STAGES) / 2.0) + DISTANCE_BETWEEN_STAGES, 0.0);
        elevatorRightStage1 = canvas.getRoot("ElevatorRightStage1", ((WIDTH + LENGTH_BETWEEN_STAGES) / 2.0) - DISTANCE_BETWEEN_STAGES, 0.0);

        elevatorLeftStage1.append(new MechanismLigament2d("Stage1Left", STAGE1_HEIGHT, 90, STAGE_WIDTH, new Color8Bit(Color.kRed)));
        elevatorRightStage1.append(new MechanismLigament2d("Stage1Right", STAGE1_HEIGHT, 90, STAGE_WIDTH, new Color8Bit(Color.kRed)));

        elevatorLeftStage2 = canvas.getRoot("ElevatorLeftStage2", ((WIDTH - LENGTH_BETWEEN_STAGES) / 2.0) + (2 * DISTANCE_BETWEEN_STAGES), STAGE2_Y); // 2.044
        elevatorRightStage2 = canvas.getRoot("ElevatorRightStage2", ((WIDTH + LENGTH_BETWEEN_STAGES) / 2.0) - (2 * DISTANCE_BETWEEN_STAGES), STAGE2_Y);

        elevatorLeftStage2.append(new MechanismLigament2d("Stage2Left", STAGE2_HEIGHT, 90, STAGE_WIDTH, new Color8Bit(Color.kRed)));
        elevatorRightStage2.append(new MechanismLigament2d("Stage2Right", STAGE2_HEIGHT, 90, STAGE_WIDTH, new Color8Bit(Color.kRed)));

        elevatorLeftStage3 = canvas.getRoot("ElevatorLeftStage3", ((WIDTH - LENGTH_BETWEEN_STAGES) / 2.0) + (3 * DISTANCE_BETWEEN_STAGES), STAGE3_Y);
        elevatorRightStage3 = canvas.getRoot("ElevatorRightStage3", ((WIDTH + LENGTH_BETWEEN_STAGES) / 2.0) - (3 * DISTANCE_BETWEEN_STAGES), STAGE3_Y);

        elevatorLeftStage3.append(new MechanismLigament2d("Stage3Left", STAGE3_HEIGHT, 90, STAGE_WIDTH, new Color8Bit(Color.kRed)));
        elevatorRightStage3.append(new MechanismLigament2d("Stage3Right", STAGE3_HEIGHT, 90, STAGE_WIDTH, new Color8Bit(Color.kRed)));

        SmartDashboard.putData("ElevatorVisualizer", canvas);
    }

    /**
     * Updates the cascading elevator visual.
     * @param positionMeters Total elevator height in meters.
     */
    public void updateElevator(double positionMeters) {
        // for each stage, from 1-3
        // each subsequent stage moves at double the speed of the previous one and half the torque
        // the fastest stage would be stage 3, the carriage
        double stage3Goal = positionMeters;
        moveStage(3, stage3Goal);
    }

    private void moveStage(int stage, double y) {
        switch (stage) {
            case 1 -> {
                elevatorLeftStage1.setPosition(((WIDTH + LENGTH_BETWEEN_STAGES) / 2.0), MathUtil.clamp(y, 0, Double.MAX_VALUE));
                elevatorRightStage1.setPosition(((WIDTH + LENGTH_BETWEEN_STAGES) / 2.0), MathUtil.clamp(y, 0, Double.MAX_VALUE));
            }
            case 2 -> {
                elevatorLeftStage2.setPosition(((WIDTH - LENGTH_BETWEEN_STAGES) / 2.0) + (2 * DISTANCE_BETWEEN_STAGES), MathUtil.clamp(y, STAGE2_Y, Double.MAX_VALUE));
                elevatorRightStage2.setPosition(((WIDTH - LENGTH_BETWEEN_STAGES) / 2.0) - (2 * DISTANCE_BETWEEN_STAGES), MathUtil.clamp(y, STAGE2_Y, Double.MAX_VALUE));
            }
            case 3 -> {
                elevatorLeftStage3.setPosition(((WIDTH - LENGTH_BETWEEN_STAGES) / 2.0) + (3 * DISTANCE_BETWEEN_STAGES), MathUtil.clamp(y, STAGE3_Y, Double.MAX_VALUE));
                elevatorRightStage3.setPosition(((WIDTH + LENGTH_BETWEEN_STAGES) / 2.0) - (3 * DISTANCE_BETWEEN_STAGES), MathUtil.clamp(y, STAGE3_Y, Double.MAX_VALUE));
            }
        }
    }
}
