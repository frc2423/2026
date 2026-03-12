package frc.robot.telemetry;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.utils.HubTracker;
import frc.robot.utils.HubTracker.Shift;

public class DashboardTelemetry {

    @Logged
    private String currentShift = "";

    @Logged
    private boolean isAllianceActive = true;

    @Logged
    private boolean isAllianceActiveNextShift = true;

    @Logged
    private double shiftTimeRemaining = 0; // seconds

    @Logged
    private double matchTime = 160; // seconds

    @Logged
    private String activeAlliance = "both";

    private Notifier notifier = new Notifier(this::update);

    public DashboardTelemetry() {
        notifier.startPeriodic(.02);
    }

    private void updateActiveAlliance() {
        boolean isRedActive = HubTracker.isActive(Alliance.Red);
        boolean isBlueActive = HubTracker.isActive(Alliance.Blue);

        if (isRedActive && isBlueActive) {
            activeAlliance = "both";
        } else if (isRedActive) {
            activeAlliance = "red";
        } else {
            activeAlliance = "blue";
        }
    }

    private void updateCurrentShift() {
        Shift shift = HubTracker.getCurrentShift();
        currentShift = shift.getShiftName();
    }

    private void updateMatchTime() {
        if (HubTracker.getCurrentShift() == Shift.AUTO) {
            matchTime = 20 - HubTracker.getMatchTime();
        } else {
            matchTime = 160 - HubTracker.getMatchTime();
        }
    }

    private void update() {
        isAllianceActive = HubTracker.isActive();
        isAllianceActiveNextShift = HubTracker.isActiveNext();
        shiftTimeRemaining = HubTracker.timeRemainingInCurrentShiftInSeconds();

        updateActiveAlliance();
        updateCurrentShift();
        updateMatchTime();

    }

}
