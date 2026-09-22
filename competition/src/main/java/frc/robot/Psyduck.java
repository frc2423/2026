package frc.robot;

// import org.junit.rules.Timeout;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
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
            
        final SwerveRequest.FieldCentricFacingAngle m_driveDistanceRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(1,0,0);
        LlmCommands.register("turn_degrees")
            .description("Turn the robot in degrees relative to the field. A negative number means counterclockwise direction. Stops within 1 degree of the target angle.")
            .doubleParam("degrees","Signed degrees to turn.",-180.0, 180.0)
            .timeout(20.0)
            .command(p -> m_robotContainer.drivetrain.applyRequest(() -> m_driveDistanceRequest.withTargetDirection(Rotation2d.fromDegrees(p.getDouble("degrees")))));
    }

    public void periodic() {
        LlmCommands.getInstance().periodic();
        LlmCommands.publishState("drivetrain/x_meters", m_robotContainer.drivetrain.getPose().getX());
        LlmCommands.publishState("drivetrain/y_meters", m_robotContainer.drivetrain.getPose().getY());
        LlmCommands.publishState("drivetrain/heading_degrees", m_robotContainer.drivetrain.getPose().getRotation().getDegrees());
    }
}
