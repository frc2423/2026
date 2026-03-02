package frc.robot.telemetry;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotContainer;

public class DashboardTelemetry {

    private final RobotContainer robot;

    @Logged
    private boolean isRedActive = false;  
    private Notifier notifier = new Notifier(this::update);

    

    public SubsystemMechanism2d(RobotContainer robot) {
        this.robot = robot;

        notifier.startPeriodic(.02);

    }

    private void update() {
        isRedActive = HubTracker.
    }
}
