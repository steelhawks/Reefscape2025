package org.steelhawks.util

import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import org.steelhawks.Robot
import org.steelhawks.RobotContainer
import org.steelhawks.subsystems.elevator.ElevatorConstants

object OperatorDashboard : VirtualSubsystem() {
    private var ntInstance: NetworkTableInstance = NetworkTableInstance.getDefault()
    private val status: NetworkTable = ntInstance.getTable("status")

    // state values to send to client these SHOULD be updated in periodic
    private val robotState: StringPublisher = status.getStringTopic("robotState").publish()
    private val elevatorLevel: StringPublisher = status.getStringTopic("elevatorLevel").publish()
    private val alliance: StringPublisher = status.getStringTopic("alliance").publish()

//    init {
//        try {
//            val ip = InetAddress.getLocalHost()
//            println("IP ADDR NT4 " + ip.hostAddress + ":2601")
//            DriverStation.reportWarning("IP Address NT4 " + ip.hostAddress + ":2601", false)
//
//            Alert("Operator Dashboard at " + ip.hostAddress + ":2601", AlertType.kInfo).set(true)
//        } catch (e: Exception) {
//            e.printStackTrace()
//        }
//    }

    fun initialize() {
        for (connection in NetworkTableInstance.getDefault().connections) {
            println("Connection: $connection")
            Alert("Connected Devices " + connection.remote_ip, AlertType.kInfo).set(true)
        }
    }

    private var counter = 0
    override fun periodic() {
        counter = (counter + 1) % 1000

        if (counter % 10 == 0) { // runs this every 10 cycles every 200 ms
            elevatorLevel.set(
                if (RobotContainer.s_Elevator.atHome().asBoolean) {
                    "Home"
                } else {
                    when (RobotContainer.s_Elevator.desiredState) {
                        ElevatorConstants.State.L1.radians -> "L1"
                        ElevatorConstants.State.L2.radians -> "L2"
                        ElevatorConstants.State.L3.radians -> "L3"
                        ElevatorConstants.State.L4.radians -> "L4"
                        else -> "N/A"
                    }
                })

            if (DriverStation.getAlliance().isPresent) {
                alliance.set(if (DriverStation.getAlliance().equals(Alliance.Red)) "Red" else "Blue")
            }

            robotState.set(Robot.getState().name)
        }
    }

    fun close() {
        robotState.close()
        elevatorLevel.close()
        alliance.close()
    }
}