package frc.robot.telemetry;

import java.util.Optional;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotContainer;
import frc.robot.utils.HubTracker;

public class DashboardTelemetry {

    private final RobotContainer robot;

    @Logged
    private int currentShift = 0;

    @Logged
    private boolean isAllianceActive = true;
    @Logged
    public double shiftTimeRemaining = 0; // seconds
    // private String activeAlliance = null;
    @Logged
    private double matchTime = 160; // seconds
    @Logged
    private String activeAlliance = "both";

    private Notifier notifier = new Notifier(this::update);

    

    public DashboardTelemetry(RobotContainer robot) {
        this.robot = robot;

        notifier.startPeriodic(.02);

    }

    private void update() {
        String[] shiftTimes = {"30-55", "55-80", "80-105", "105-130"};
        for (int i = 0; i < 4; i++) {
            String[] shiftRange = shiftTimes[i].split("-");
            if (matchTime <= 30) {
                activeAlliance = "both";
                currentShift = 5;
            }
            else if (DriverStation.getAlliance() == Optional.of(Alliance.Red) && currentShift >= 1) {
                if (isAllianceActive) {
                    activeAlliance = "red";
                } else {
                    activeAlliance = "blue";
                }
            } else if (currentShift >= 1) {
                if (isAllianceActive) {
                    activeAlliance = "blue";
                } else {
                    activeAlliance = "red";
                }
            }
            // System.out.println(HubTracker.getMatchTime());
            matchTime = 160 - HubTracker.getMatchTime();
                if (HubTracker.getMatchTime() >= Integer.parseInt(shiftRange[0]) && HubTracker.getMatchTime() <= Integer.parseInt(shiftRange[1])) {
                    /// if we're in a shift, run shift code updates
                    // @Logged 
                    currentShift = i + 1;
                    isAllianceActive = HubTracker.isActive();
                    shiftTimeRemaining = HubTracker.timeRemainingInCurrentShiftInSeconds();
                }
        }

        
    }

}
