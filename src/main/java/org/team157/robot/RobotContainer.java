// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.team157.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team157.robot.commands.DriveCommands;
import org.team157.robot.generated.TunerConstants;
import org.team157.robot.subsystems.drive.Drive;
import org.team157.robot.subsystems.drive.GyroIO;
import org.team157.robot.subsystems.drive.GyroIOPigeon2;
import org.team157.robot.subsystems.drive.ModuleIO;
import org.team157.robot.subsystems.drive.ModuleIOSim;
import org.team157.robot.subsystems.drive.ModuleIOTalonFX;
import org.team157.robot.subsystems.flywheel.Flywheel;
import org.team157.robot.subsystems.flywheel.FlywheelConstants;
import org.team157.robot.subsystems.flywheel.FlywheelIOSparkflex;
import org.team157.robot.subsystems.hood.Hood;
import org.team157.robot.subsystems.hood.HoodConstants;
import org.team157.robot.subsystems.hood.HoodIOSparkMax;
import org.team157.robot.subsystems.hopper.Hopper;
import org.team157.robot.subsystems.hopper.HopperIOTalonFX;
import org.team157.robot.subsystems.intake.Intake;
import org.team157.robot.subsystems.intake.IntakeIOTalonFX;
import org.team157.robot.subsystems.uptake.Uptake;
import org.team157.robot.subsystems.uptake.UptakeIOSparkMax;
import org.team157.robot.subsystems.vision.Vision;
import org.team157.robot.subsystems.vision.VisionConstants;
import org.team157.robot.subsystems.vision.VisionIO;
import org.team157.robot.subsystems.vision.VisionIOPhotonVision;
import org.team157.robot.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public static Vision vision;
  public static Drive drive;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public static boolean dumperMode = false;
  public Hood hood = new Hood();
  public Flywheel flywheel = new Flywheel();
  public Hopper hopper = new Hopper();
  public Intake intake = new Intake();
  public Uptake uptake = new Uptake();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Set up the mechanism subsystems. Must happen before configureButtonBindings, since their
    // commands are built from the IO layer.
    hood.setIO(new HoodIOSparkMax(hood));
    hood.setDefaultCommand(hood.getDefault());
    intake.setIO(new IntakeIOTalonFX(intake));
    intake.setDefaultCommand(intake.getDefault());
    hopper.setIO(new HopperIOTalonFX(hopper));
    hopper.setDefaultCommand(hopper.getDefault());
    uptake.setIO(new UptakeIOSparkMax(uptake));
    uptake.setDefaultCommand(uptake.getDefault());
    flywheel.setIO(new FlywheelIOSparkflex(flywheel), vision);
    flywheel.setDefaultCommand(flywheel.getDefault());
    autoChooser.addOption("Flywheel SysId", flywheel.sysId());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    // Keep the angle and distance to the target up to date while no other vision commands are
    // running.
    vision.setDefaultCommand(vision.setDefault(drive));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    //////////////////////////////////////////////
    ///             DRIVER COMMANDS            ///
    //////////////////////////////////////////////
    // Toggle Dumper Mode by pressing Start and Back together
    controller.start().and(controller.back()).onTrue(toggleDumperMode());

    // Shoot while held. In Dumper Mode the back of the robot also turns to face the target, and the
    // flywheel speed and hood angle are calculated from the distance to the target.
    controller.rightTrigger().and(dumperModeTrigger().negate()).whileTrue(manualShot());
    controller.rightTrigger().and(dumperModeTrigger()).whileTrue(dumperShot());

    // Run the intake while held
    controller.leftTrigger().whileTrue(intake.set(0.5));

    // TODO: temporary hood direction check, hold Start and the hood angle should increase
    // (Hood/AngleDegrees goes up). Hold Back to lower it. Ignored while both are held, since that
    // toggles Dumper Mode.
    controller.start().and(controller.back().negate()).whileTrue(hood.set(0.05));
    controller.back().and(controller.start().negate()).whileTrue(hood.set(-0.05));
    // TODO: temporary closed loop test, click a stick to move the hood to its min or max angle
    controller.leftStick().onTrue(hood.setAngle(HoodConstants.LOWER_SOFT_LIMIT));
    controller.rightStick().onTrue(hood.setAngle(HoodConstants.UPPER_SOFT_LIMIT));
  }

  /**
   * Returns the current state of Dumper Mode.
   *
   * @return a {@link Trigger} with the current state of Dumper Mode
   */
  private Trigger dumperModeTrigger() {
    return new Trigger(() -> (dumperMode));
  }

  /** Turns Dumper Mode on or off. */
  private Command toggleDumperMode() {
    return Commands.runOnce(
            () -> {
              dumperMode = !dumperMode;
              Logger.recordOutput("DumperMode", dumperMode);
            })
        .ignoringDisable(true);
  }

  /**
   * Shoots at a fixed flywheel speed, without aiming. Feeds balls once the flywheel is up to speed.
   */
  private Command manualShot() {
    return Commands.parallel(
        flywheel.setVelocity(FlywheelConstants.MANUAL_SHOT_VELOCITY),
        feedWhenReady(flywheel::isAtTargetVelocity));
  }

  /**
   * Turns the back of the robot to face the target and sets the flywheel speed and hood angle for
   * the distance to the target. Feeds balls once the flywheel is up to speed and the robot is
   * aimed. The robot holds still while aiming, since shooting on the move is not supported.
   */
  private Command dumperShot() {
    return Commands.parallel(
        DriveCommands.joystickDriveAtAngle(
            drive, () -> 0, () -> 0, vision::getDriveAngleToFaceTarget),
        flywheel.setDynamicVelocity(),
        hood.setAngle(flywheel::getDesiredHoodAngle),
        feedWhenReady(() -> flywheel.isAtTargetVelocity() && vision.isAimed()));
  }

  /**
   * Waits until ready to shoot, then runs the hopper and uptake to feed balls into the flywheel.
   * Feeds anyway after {@link FlywheelConstants#SPIN_UP_TIMEOUT}, so a slow flywheel or aim can't
   * stop the robot from shooting.
   *
   * @param ready Whether the robot is ready to shoot.
   */
  private Command feedWhenReady(BooleanSupplier ready) {
    return Commands.waitUntil(ready)
        .withTimeout(FlywheelConstants.SPIN_UP_TIMEOUT.in(Seconds))
        .andThen(Commands.parallel(hopper.set(0.5), uptake.set(0.5)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
