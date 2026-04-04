package frc.robot.telemetry;

import java.util.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.utils.HubTracker;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TwindexerSubsystem;

public class RobotHealth {

    private CommandSwerveDrivetrain drivetrain;

    private Vision vision;// = new Vision(() -> drivetrain.getState().Pose);

    private TwindexerSubsystem twindexer;
    
    @Logged
    boolean isCameraConnected = vision.isCameraConnected();

    @Logged
    private Map<String, HashMap<String, Boolean>> motorLogs = new HashMap<>();

    // {
    //     "twindexer": {
    //         "totalAmpsTooHigh": false;
    //         "voltage": 2;
    //     }
    // }

   


    private Notifier notifier = new Notifier(this::update);

    public RobotHealth(Vision vision, CommandSwerveDrivetrain drivetrain, TwindexerSubsystem twindexer) {
        notifier.startPeriodic(.02);
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.twindexer = twindexer;

        motorLogs.put("twindexer", new Map.of(
            "twindexer", new HashMap<String, Boolean>(),
            "feederRight", new HashMap<String,Boolean>(),
            "feederLeft", new HashMap<String,Boolean>()

            

        );
        
        
        ({"isConnected", twindexer.getSparkFlex().getFirmwareVersion()?true : false}));
        
    }

     private static void addDefinition(Map<String, List<String>> dict, String word, String definition) {
        dict.computeIfAbsent(word, k -> new ArrayList<>()).add(definition);
    }

    private void update() {
        
        isCameraConnected = vision.isCameraConnected();

    }

}


