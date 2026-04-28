package frc.robot.telemetry;

import java.util.*;

import frc.robot.NTHelper;
import frc.robot.RobotContainer;

public class RobotHealth {

    private RobotContainer robot;
    // @Logged
    // boolean isCameraConnected = vision.isCameraConnected();

    //@Logged
    private HashMap<String, HashMap<String, Boolean>> motorLogs = new HashMap<>();

    // {
    //     "twindexer": {
    //         "totalAmpsTooHigh": false;
    //         "voltage": 2;
    //     }
    // }


    public RobotHealth(RobotContainer robot) {
        this.robot = robot;
        
        motorLogs.put("twindexer", new HashMap<String, Boolean>());
        motorLogs.put( "feederRight", new HashMap<String,Boolean>());
        motorLogs.put("feederLeft", new HashMap<String,Boolean>());
        motorLogs.put("intake", new HashMap<String, Boolean>());
        motorLogs.put("shooterLeft", new HashMap<String, Boolean>());
        motorLogs.put("shooterRight", new HashMap<String, Boolean>());
        motorLogs.put("hood", new HashMap<String, Boolean>());
        motorLogs.put("arm", new HashMap<String, Boolean>());
        motorLogs.put("intake", new HashMap<String, Boolean>());
    }

    public void update() {
        
        //isCameraConnected = vision.isCameraConnected();
        motorLogs.get("twindexer").put("isConnected", isConnected(robot.twindexer.getSparkFlex().getBusVoltage()));
        motorLogs.get("intake").put("isOneConnected", isConnected(robot.intake.getSparkFlex1().getBusVoltage()));
        motorLogs.get("intake").put("isTwoConnected", isConnected(robot.intake.getSparkFlex2().getBusVoltage()));
        motorLogs.get("shooterLeft").put("isConnected", isConnected(robot.shooterLeft.getSparkFlex().getBusVoltage()));
        motorLogs.get("shooterRight").put("isConnected", isConnected(robot.shooterRight.getSparkFlex().getBusVoltage()));
        motorLogs.get("hood").put("isConnected", isConnected(robot.hood.getSparkMax().getBusVoltage()));
        motorLogs.get("arm").put("isConnected", isConnected(robot.arm.getSparkMax().getBusVoltage()));
        motorLogs.get("feederLeft").put("isConnected", isConnected(robot.feederLeft.getSparkFlex().getBusVoltage()));
        motorLogs.get("feederRight").put("isConnected", isConnected(robot.feederRight.getSparkFlex().getBusVoltage()));

        motorLogs.get("intake").put("isStalled", robot.intake.isStalled()==true);
        motorLogs.get("hood").put("isStalled", robot.hood.isStalled()==true);
        motorLogs.get("feederLeft").put("isStalled", robot.feederLeft.isStalled()==true);
        motorLogs.get("feederRight").put("isStalled", robot.feederRight.isStalled()==true);
        motorLogs.get("twindexer").put("isStalled", robot.twindexer.isJammed()==true);

        log();
    }

    private void log() {
        for (Map.Entry<String, HashMap<String, Boolean>> set : motorLogs.entrySet()) {
            String subsystemName = set.getKey();
            for (Map.Entry<String,Boolean> value : set.getValue().entrySet()) {
                String field = value.getKey();
                String key = "/RobotHealth/" + subsystemName + "/" + field;
                NTHelper.setBoolean(key,value.getValue()); 
            }
        }

        // double voltage = robot.feederRight.getSparkFlex().getBusVoltage();

        // System.out.println("bus voltage: " + isConnected(voltage) + ", " + voltage);

        // NTHelper.setDouble("/RobotHealth/feederRight/busVoltage", robot.feederRight.getSparkFlex().getBusVoltage());
    }

    private boolean isConnected(double voltage) {
        if (Double.isNaN(voltage)) {
            return false;
        }
        if (Math.abs(voltage) < .001) {
            return false;
        }
        return true;
    }

}


