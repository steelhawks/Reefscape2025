from edu.wpi.first.wpilibj import Timer
from edu.wpi.first.math.kinematics import ChassisSpeeds

from edu.wpi.first.wpilibj2.command import CommandBase, SequentialCommandGroup, ParallelCommandGroup, WaitCommand, WaitUntilCommand, ConditionalCommand, RunCommand, InstantCommand, CommandScheduler
from org.steelhawks.subsystems.elevator import ElevatorConstants
from org.steelhawks import RobotContainer

class RC2Pathless(AutoRoutine):
    RUNNING_OUT_OF_TIME = 13.0
    AUTO_ALIGNMENT_TIMEOUT = 2.0  # example timeout, adjust as per your Java constant
    ELEVATOR_TIMEOUT = 2.0  # example timeout, adjust as needed
    WAIT_FOR_CORAL_TIMEOUT = 3.0
    WAIT_FOR_CORAL_TIMEOUT_LAST_EFFORT = 2.0

    def __init__(self, startingBR1: bool, desiredScoreLevel):
        super().__init__()

        auton_time = Timer()

        def start_pose():
            pose = ReefUtil.AllianceFlip.apply(ReefUtil.StartEndPosition.RC2.get_pose())
            s_Swerve.set_pose(pose)
            auton_time.reset()
            auton_time.start()

        def follow_trajectory(name):
            return Autos.follow_trajectory(name)

        def shoot_coral_end():
            return s_Claw.shoot_coral_end()

        # Commands from Java converted to Python style
        self.addCommands(
            InstantCommand(start_pose),

            ParallelCommandGroup(
                RunCommand(lambda: follow_trajectory(f"RC2 to BR{'1' if startingBR1 else '2'}"), interruptible=True),
                SequentialCommandGroup(
                    WaitCommand(0.6),
                    s_Elevator.set_desired_state(desiredScoreLevel)
                )
            ),

            # SwerveDriveAlignment with timeout
            SwerveDriveAlignment(
                ReefUtil.CoralBranch.BR1.get_score_pose(desiredScoreLevel) if startingBR1 else ReefUtil.CoralBranch.BR2.get_score_pose(desiredScoreLevel)
            ).withTimeout(self.AUTO_ALIGNMENT_TIMEOUT),

            # Elevator deadline
            ParallelCommandGroup(
                WaitCommand(self.ELEVATOR_TIMEOUT),
                WaitUntilCommand(lambda: s_Elevator.at_this_goal(desiredScoreLevel))
            ),

            shoot_coral_end(),
            s_Elevator.set_desired_state(ElevatorConstants.State.HOME),

            # Drive to Lower Source
            RunCommand(lambda: follow_trajectory(f"BR{'1' if startingBR1 else '2'} to Lower Source"), interruptible=True),

            WaitUntilCommand(s_Claw.has_coral).withTimeout(self.WAIT_FOR_CORAL_TIMEOUT),

            ConditionalCommand(
                # If timed out: backup and try again
                RunCommand(
                    lambda: s_Swerve.run_velocity(
                        ChassisSpeeds(
                            -0.3 * FieldConstants.CoralStation.BOTTOM.get_intake_pose().rotation.cos(),
                            -0.3 * FieldConstants.CoralStation.BOTTOM.get_intake_pose().rotation.sin(),
                            0.0
                        )
                    )
                ).until(s_Swerve.is_stalling).until(s_Claw.has_coral).withTimeout(self.WAIT_FOR_CORAL_TIMEOUT_LAST_EFFORT),
                # else do nothing
                InstantCommand(),
                s_Claw.has_coral
            ),

            DriveCommands.drive_to_position(ReefUtil.CoralBranch.BL2.get_auton_slow_drive_pose(desiredScoreLevel)),
            s_Elevator.set_desired_state(desiredScoreLevel),

            SwerveDriveAlignment(lambda: ReefUtil.CoralBranch.BL2.get_score_pose(desiredScoreLevel)),

            ParallelCommandGroup(
                WaitCommand(self.ELEVATOR_TIMEOUT),
                WaitUntilCommand(lambda: s_Elevator.at_this_goal(desiredScoreLevel))
            ),

            shoot_coral_end(),
            s_Elevator.set_desired_state(ElevatorConstants.State.HOME),

            RunCommand(lambda: follow_trajectory("BL2 to Lower Source"), interruptible=True),

            WaitUntilCommand(s_Claw.has_coral).withTimeout(self.WAIT_FOR_CORAL_TIMEOUT),

            ConditionalCommand(
                RunCommand(
                    lambda: s_Swerve.run_velocity(
                        ChassisSpeeds(
                            -0.3 * FieldConstants.CoralStation.BOTTOM.get_intake_pose().rotation.cos(),
                            -0.3 * FieldConstants.CoralStation.BOTTOM.get_intake_pose().rotation.sin(),
                            0.0
                        )
                    )
                ).until(s_Swerve.is_stalling).until(s_Claw.has_coral).withTimeout(self.WAIT_FOR_CORAL_TIMEOUT_LAST_EFFORT),
                InstantCommand(),
                s_Claw.has_coral
            ),

            DriveCommands.drive_to_position(ReefUtil.CoralBranch.BL1.get_auton_slow_drive_pose(desiredScoreLevel)),
            s_Elevator.set_desired_state(desiredScoreLevel),

            SwerveDriveAlignment(lambda: ReefUtil.CoralBranch.BL1.get_score_pose(desiredScoreLevel)),

            ParallelCommandGroup(
                WaitCommand(self.ELEVATOR_TIMEOUT),
                WaitUntilCommand(lambda: s_Elevator.at_this_goal(desiredScoreLevel))
            ),

            shoot_coral_end(),
            s_Elevator.set_desired_state(ElevatorConstants.State.HOME),

            RunCommand(lambda: follow_trajectory("BL1 to Lower Source"), interruptible=True),

            WaitUntilCommand(s_Claw.has_coral).withTimeout(self.WAIT_FOR_CORAL_TIMEOUT),

            ConditionalCommand(
                RunCommand(
                    lambda: s_Swerve.run_velocity(
                        ChassisSpeeds(
                            -0.3 * FieldConstants.CoralStation.BOTTOM.get_intake_pose().rotation.cos(),
                            -0.3 * FieldConstants.CoralStation.BOTTOM.get_intake_pose().rotation.sin(),
                            0.0
                        )
                    )
                ).until(s_Swerve.is_stalling).until(s_Claw.has_coral).withTimeout(self.WAIT_FOR_CORAL_TIMEOUT_LAST_EFFORT),
                InstantCommand(),
                s_Claw.has_coral
            ),

            # Drive to L2 or BR1 depending on startingBR1
            RunCommand(
                lambda: DriveCommands.drive_to_position(
                    ReefUtil.CoralBranch.L2.get_auton_slow_drive_pose(desiredScoreLevel) if startingBR1
                    else ReefUtil.CoralBranch.BR1.get_auton_slow_drive_pose(desiredScoreLevel)
                )
            ),

            # Final elevator and alignment sequence
            RunCommand(lambda: None).andThen(
                InstantCommand(lambda:
                               (ElevatorConstants.State.L2 if auton_time.hasElapsed(self.RUNNING_OUT_OF_TIME) and startingBR1 else desiredScoreLevel)
                               )
            ).andThen(
                SwerveDriveAlignment(lambda:
                                     ReefUtil.CoralBranch.L2.get_score_pose(ElevatorConstants.State.L2 if auton_time.hasElapsed(self.RUNNING_OUT_OF_TIME) and startingBR1 else desiredScoreLevel)
                                     )
            ).andThen(
                ParallelCommandGroup(
                    WaitCommand(self.ELEVATOR_TIMEOUT),
                    WaitUntilCommand(lambda: s_Elevator.at_this_goal(desiredScoreLevel))
                )
            ).andThen(
                shoot_coral_end()
            ).andThen(
                s_Elevator.set_desired_state(ElevatorConstants.State.HOME)
            )
        )


