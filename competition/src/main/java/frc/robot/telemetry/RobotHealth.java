package frc.robot.telemetry;

import java.util.*;

import edu.wpi.first.wpilibj.Notifier;
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

   


    private Notifier notifier = new Notifier(this::update);

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
        
        notifier.startPeriodic(.02);
        //HashMap<String, Boolean> hashMap = new HashMap<>();
        // (Map.of(
        //     "twindexer", new HashMap<String, Boolean>(),
        //     "feederRight", new HashMap<String,Boolean>(),
        //     "feederLeft", new HashMap<String,Boolean>()
        // ));
    }

     private static void addDefinition(Map<String, List<String>> dict, String word, String definition) {
        dict.computeIfAbsent(word, k -> new ArrayList<>()).add(definition);
    }

    private void update() {
        
        //isCameraConnected = vision.isCameraConnected();
        motorLogs.get("twindexer").put("isConnected", robot.twindexer.getSparkFlex().getFirmwareVersion()!=0);
        motorLogs.get("intake").put("isConnected", robot.intake.getSparkFlex().getFirmwareVersion()!=0);
        motorLogs.get("shooterLeft").put("isConnected", robot.shooterLeft.getSparkFlex().getFirmwareVersion()!=0);
        motorLogs.get("shooterRight").put("isConnected", robot.shooterRight.getSparkFlex().getFirmwareVersion()!=0);
        motorLogs.get("hood").put("isConnected", robot.hood.getSparkMax().getFirmwareVersion()!=0);
        motorLogs.get("arm").put("isConnected", robot.arm.getSparkMax().getFirmwareVersion()!=0);
        motorLogs.get("feederLeft").put("isConnected", robot.feederLeft.getSparkFlex().getFirmwareVersion()!=0);
        motorLogs.get("feederRight").put("isConnected", robot.feederRight.getSparkFlex().getFirmwareVersion()!=0);

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
    }

}


