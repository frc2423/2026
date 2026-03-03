package frc.robot.telemetry;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotContainer;

@Logged
public class DashboardTelemetry {

    private final RobotContainer robot;

    @Logged
    private int currentShift = 0;


    private String activeAlliance = null;
    private int shiftTimeRemaining = 0; // seconds
    private int matchTime = 160; // seconds


    private Notifier notifier = new Notifier(this::update);

    

    public SubsystemMechanism2d(RobotContainer robot) {
        this.robot = robot;

        notifier.startPeriodic(.02);

    }

    private void updateCurrentShift() {
        String[] shiftTimes = {"30-55", "55-80", "80-105", "105-130"};
        for (int i = 0; i < 4; i++) {
            String[] shiftRange = shiftTimes[i].split();
            for (int j = 0; j < shiftTimes[i].split(); j++) {
                if (HubTracker.getMatchTime() >= shiftRange[0] && HubTracker.getMatchTime() <= shiftRange[1]) {
                    // @Logged 
                    currentShift = i + 1;
                }
            }
        }

        
    }
}
