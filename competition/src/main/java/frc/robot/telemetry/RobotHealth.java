package frc.robot.telemetry;

import java.util.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.utils.HubTracker;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TwindexerSubsystem;

public class RobotHealth {

    private CommandSwerveDrivetrain drivetrain;

    private Vision vision;// = new Vision(() -> drivetrain.getState().Pose);

    private TwindexerSubsystem twindexer;

    private IntakeSubsystem intake;
    
    private ShooterSubsystem shooter;
    
    private HoodSubsystem hood;

    private ArmSubsystem arm;

    private FeederSubsystem feeder;

    @Logged
    boolean isCameraConnected = vision.isCameraConnected();

    @Logged
    private HashMap<String, HashMap<String, Boolean>> motorLogs = new HashMap<>();

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
        this.intake = intake;
        this.shooter = shooter;
        this.hood = hood;
        this.arm = arm;
        this.feeder = feeder;

        motorLogs.put("twindexer", new HashMap<String, Boolean>());
        motorLogs.put( "feederRight", new HashMap<String,Boolean>());
        motorLogs.put("feederLeft", new HashMap<String,Boolean>());
        motorLogs.put("intake", new HashMap<String, Boolean>());
        motorLogs.put("shooter", new HashMap<String, Boolean>());
        motorLogs.put("hood", new HashMap<String, Boolean>());
        motorLogs.put("arm", new HashMap<String, Boolean>());
        motorLogs.put("intake", new HashMap<String, Boolean>());
        
        //HashMap<String, Boolean> hashMap = new HashMap<>();
        // (Map.of(
        //     "twindexer", new HashMap<String, Boolean>(),
        //     "feederRight", new HashMap<String,Boolean>(),
        //     "feederLeft", new HashMap<String,Boolean>()
        // ));

        motorLogs.get("twindexer").put("isConnected", twindexer.getSparkFlex().getFirmwareVersion()!=0);
        motorLogs.get("intake").put("isConnected", intake.getSparkFlex().getFirmwareVersion()!=0);
        motorLogs.get("shooter").put("isConnected", shooter.getSparkFlex().getFirmwareVersion()!=0);
        motorLogs.get("hood").put("isConnected", hood.getSparkMax().getFirmwareVersion()!=0);
        motorLogs.get("arm").put("isConnected", arm.getSparkMax().getFirmwareVersion()!=0);
        motorLogs.get("feeder").put("isConnected", feeder.getSparkFlex().getFirmwareVersion()!=0);

    }

     private static void addDefinition(Map<String, List<String>> dict, String word, String definition) {
        dict.computeIfAbsent(word, k -> new ArrayList<>()).add(definition);
    }

    private void update() {
        
        isCameraConnected = vision.isCameraConnected();

    }

}


