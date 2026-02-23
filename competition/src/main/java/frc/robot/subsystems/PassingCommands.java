package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.NTHelper;
import frc.robot.generated.FieldConstants;
import frc.robot.generated.PoseTransformUtils;

public class PassingCommands extends SubsystemBase {

    private final BLine bline;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final ShooterCommands shooter;
    private final CommandXboxController controller;

    private CommandSwerveDrivetrain swerve;

    public PassingCommands(CommandSwerveDrivetrain swerve, BLine bline,
            IntakeSubsystem intake, ArmSubsystem arm, ShooterCommands shooter, CommandXboxController controller) {
        this.swerve = swerve;
        this.bline = bline;
        this.intake = intake;
        this.arm = arm;
        this.shooter = shooter;
        this.controller = controller;
    }

    public Command trenchPass() {
        Pose2d[] trenchPosesBlue = { new Pose2d(6, 0.8, Rotation2d.fromDegrees(180)),
                new Pose2d(6, 7.3, Rotation2d.fromDegrees(180)) };
        Pose2d[] trenchPosesRed = { new Pose2d(10.5, 0.8, Rotation2d.fromDegrees(0)),
                new Pose2d(10.5, 7.3, Rotation2d.fromDegrees(0)) };

        Command driveToNearestTrench = Commands.either(bline.goToNearestPose(trenchPosesRed),
                bline.goToNearestPose(trenchPosesBlue),
                () -> PoseTransformUtils.isRedAlliance());

        Command passFuel = Commands.parallel(
                arm.armDown(),
                Commands.sequence(
                        Commands.waitUntil(() -> arm.isDown()),
                        intake.outtake()));

        return Commands.sequence(driveToNearestTrench, passFuel);
    }

    public Command shootPass() {
        Pose2d[] trenchPosesBlue = { new Pose2d(6, 1.75, Rotation2d.fromDegrees(180)),
                new Pose2d(6, 6.25, Rotation2d.fromDegrees(180)) };
        Pose2d[] trenchPosesRed = { new Pose2d(10.5, 1.75, Rotation2d.fromDegrees(0)),
                new Pose2d(10.5, 6.25, Rotation2d.fromDegrees(0)) };

        Command goToNearestPassingSpot = Commands.either(bline.goToNearestPose(trenchPosesRed),
                bline.goToNearestPose(trenchPosesBlue),
                () -> PoseTransformUtils.isRedAlliance());

        Command passFuel = Commands.parallel(
                shooter.revSpeedFromDAS(),
                Commands.sequence(
                        Commands.waitSeconds(3),
                        shooter.spinFeeder(() -> {
                            return NTHelper.getDouble("/tuning/FeederSpeed", 0);
                        })));

        return Commands.sequence(goToNearestPassingSpot, passFuel);
    }

    public Command aimToPass() {
        return Commands.parallel(
        shooter.actuallyLookAngle(() -> {
            Pose2d targetPose = Pose2d.kZero;
            if (swerve.getPose().getY() > FieldConstants.LinesHorizontal.center) {
                if (PoseTransformUtils.isRedAlliance()) {
                    targetPose = new Pose2d(FieldConstants.fieldLength, 6.5 + 1.5 * controller.getRightX(),
                            Rotation2d.fromDegrees(180));
                } else {
                    targetPose = new Pose2d(0, 6.5 - 1.5 * controller.getRightX(), Rotation2d.fromDegrees(180));
                }
            } else {
                if (PoseTransformUtils.isRedAlliance()) {
                    targetPose = new Pose2d(FieldConstants.fieldLength, 1.5 + 1.5 * controller.getRightX(),
                            Rotation2d.fromDegrees(180));
                } else {
                    targetPose = new Pose2d(0, 1.5 - 1.5 * controller.getRightX(), Rotation2d.fromDegrees(180));
                }
            }
            return targetPose;
        }),
        shooter.rev(() -> 3200.0));
    }

}