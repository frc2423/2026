package frc.robot;

import frc.robot.llm.LlmCommands;

/** Exposes robot commands and telemetry to the Psyduck LLM client over NetworkTables. */
public class Psyduck {

    private final RobotContainer m_robotContainer;

    public Psyduck(RobotContainer robotContainer) {
        m_robotContainer = robotContainer;
        configureLlmCommands();
    }

    private void configureLlmCommands() {
        LlmCommands.register("drive_distance")
                .description(
                        "Drive the robot in a straight line along its current heading. Positive distance"
                                + " drives forward, negative drives backward. Finishes when the distance is"
                                + " reached.")
                .doubleParam("meters", "Signed distance to travel in meters.", -10.0, 10.0)
                .timeout(20.0)
                .command(p -> m_robotContainer.drivetrain.driveToDistanceCommand(p.getDouble("meters"), 1.0));

        LlmCommands.register("rotation")
                .description(
                    "Have the robot rotate a certain number of degrees"
                )
                .doubleParam("degrees", "how much do you want the robot to turn in degrees", -360.0, 360.0)
                .timeout(10.0)
                .command(p -> m_robotContainer.drivetrain.rotateToDegreesCommand(p.getDouble("degrees"), 10.0));
        
    }

    public void periodic() {
        LlmCommands.getInstance().periodic();
        LlmCommands.publishState("drivetrain/x_meters", m_robotContainer.drivetrain.getPose().getX());
        LlmCommands.publishState("drivetrain/y_meters", m_robotContainer.drivetrain.getPose().getY());
    }
}
