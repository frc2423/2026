package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.NTHelper;
import frc.robot.RobotContainer;
import frc.robot.generated.FieldConstants;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.FlippingUtil;

public class PassingCommands extends SubsystemBase {

    private BLine bline;
    private IntakeSubsystem intake;
    private ArmSubsystem arm;
    private ShooterCommands shooter;
    private CommandXboxController controller;

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
        return Commands.sequence(
                Commands.either(bline.goToNearestPose(trenchPosesRed), bline.goToNearestPose(trenchPosesBlue),
                        () -> PoseTransformUtils.isRedAlliance()),
                arm.setAngle(Degrees.of(10)),
                intake.outtake());
    }

    public Command shootPass() {
        Pose2d[] trenchPosesBlue = { new Pose2d(6, 1.75, Rotation2d.fromDegrees(180)),
                new Pose2d(6, 6.25, Rotation2d.fromDegrees(180)) };
        Pose2d[] trenchPosesRed = { new Pose2d(10.5, 1.75, Rotation2d.fromDegrees(0)),
                new Pose2d(10.5, 6.25, Rotation2d.fromDegrees(0)) };
        return Commands.sequence(
                Commands.either(bline.goToNearestPose(trenchPosesRed), bline.goToNearestPose(trenchPosesBlue),
                        () -> PoseTransformUtils.isRedAlliance()),
                shooter.revSpeedFromDAS().withTimeout(3),
                shooter.spinFeeder(() -> {
                    return NTHelper.getDouble("/tuning/FeederSpeed", 0);
                }));
    }

    public Command aimToPass() {
        return shooter.actuallyLookAngle(() -> {
            Pose2d targetPose = Pose2d.kZero;
            if (swerve.getPose().getY() > FieldConstants.LinesHorizontal.center) {
                if (PoseTransformUtils.isRedAlliance()) {
                    targetPose = new Pose2d(FieldConstants.fieldLength, 6.5 + 1.5*controller.getRightX(), Rotation2d.fromDegrees(180));
                } else {
                    targetPose = new Pose2d(0, 6.5 - 1.5*controller.getRightX(), Rotation2d.fromDegrees(180));
                }
            } else {
                if (PoseTransformUtils.isRedAlliance()) {
                    targetPose = new Pose2d(FieldConstants.fieldLength, 1.5 + 1.5*controller.getRightX(), Rotation2d.fromDegrees(180));
                } else {
                    targetPose = new Pose2d(0, 1.5 - 1.5*controller.getRightX(), Rotation2d.fromDegrees(180));
                }
            }
            return targetPose;
        });
    }


}