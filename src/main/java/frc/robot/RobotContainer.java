package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.pipeline.DriveInput;
import frc.robot.commands.pipeline.DriveInputPipeline;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.dashboard.DriverDashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleConstants;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.CameraIOSim;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.utility.Elastic;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final AprilTagVision vision;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Alerts for controller disconnection
  private final Alert driverDisconnected =
      new Alert(
          String.format(
              "Driver xbox controller disconnected (port %s).",
              driverController.getHID().getPort()),
          AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert(
          String.format(
              "Operator xbox controller disconnected (port %s).",
              operatorController.getHID().getPort()),
          AlertType.kWarning);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Alerts
  private final Alert notPrimaryBotAlert =
      new Alert("Robot type is not the primary robot type.", AlertType.kInfo);
  private final Alert tuningModeActiveAlert =
      new Alert("Tuning mode active, do not use in competition.", AlertType.kWarning);
  private static final Alert testPlansAvailable =
      new Alert(
          "Running with test plans enabled, ensure you are using the correct auto.",
          AlertType.kWarning);

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (Constants.getRobot()) {
      case PHOENIX_TUNER_X:
        // Real robot (Competition bot with mechanisms), instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(DriveConstants.GYRO_CAN_ID, true),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision = new AprilTagVision();
        break;

      case CHASSIS_2025:
        // Real robot (Competition bot with mechanisms), instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(DriveConstants.GYRO_CAN_ID, false),
                new ModuleIOSparkMax(ModuleConstants.FRONT_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.FRONT_RIGHT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.BACK_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(ModuleConstants.BACK_RIGHT_MODULE_CONFIG));
        vision = new AprilTagVision();
        break;

      case SIM_BOT:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new AprilTagVision(
                new CameraIOSim(VisionConstants.SIM_FRONT_CAMERA, drive::getRobotPose));
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
        vision = new AprilTagVision();
        break;
    }

    // Vision setup
    vision.setLastRobotPoseSupplier(drive::getRobotPose);
    vision.addVisionEstimateConsumer(
        (estimate) -> {
          if (estimate.status().isSuccess() && Constants.getMode() != Mode.SIM) {
            drive.addVisionMeasurement(
                estimate.robotPose().toPose2d(),
                estimate.timestampSeconds(),
                estimate.standardDeviations());
          }
        });

    // Can also use AutoBuilder.buildAutoChooser(); instead of SendableChooser to auto populate
    registerNamedCommands();
    // autoChooser = new LoggedDashboardChooser<>("Auto Chooser", new SendableChooser<Command>());
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("None", Commands.none());

    // Configure autos
    configureAutos(autoChooser);

    // Alerts for constants to avoid using them in competition
    tuningModeActiveAlert.set(Constants.TUNING_MODE);
    testPlansAvailable.set(Constants.RUNNING_TEST_PLANS);
    notPrimaryBotAlert.set(Constants.getRobot() != Constants.PRIMARY_ROBOT_TYPE);

    // Hide controller missing warnings for sim
    DriverStation.silenceJoystickConnectionWarning(Constants.getMode() != Mode.REAL);

    initDashboard();

    // Configure the button bindings
    configureControllerBindings();
  }

  /** Configure drive dashboard object */
  private void initDashboard() {
    SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());

    DriverDashboard.poseSupplier = drive::getRobotPose;
    DriverDashboard.speedsSupplier = drive::getRobotSpeeds;
    DriverDashboard.hasVisionEstimate = vision::hasVisionEstimate;
    DriverDashboard.currentDriveModeName =
        () -> drive.getCurrentCommand() == null ? "Idle" : drive.getCurrentCommand().getName();

    DriverDashboard.addCommand("Reset Pose", () -> drive.resetPose(new Pose2d()), true);
    DriverDashboard.addCommand(
        "Reset Rotation",
        drive.runOnce(
            () ->
                drive.resetPose(
                    new Pose2d(drive.getRobotPose().getTranslation(), Rotation2d.kZero))),
        true);
  }

  public void updateAlerts() {
    // Controller disconnected alerts
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driverController.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driverController.getHID().getPort()));
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operatorController.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(operatorController.getHID().getPort()));
  }

  /** Define button->command mappings. */
  private void configureControllerBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    configureDriverControllerBindings(driverController);
    configureOperatorControllerBindings(operatorController);
    configureAlertTriggers();
  }

  private void configureDriverControllerBindings(CommandXboxController xbox) {

    final DriveInputPipeline pipeline =
        new DriveInputPipeline(
            new DriveInput(drive, "Drive")
                .translationStick(() -> -xbox.getLeftY(), () -> -xbox.getLeftX())
                .rotationStick(() -> -xbox.getRightX())
                .fieldRelativeEnabled());

    // Default command, normal joystick drive
    drive.setDefaultCommand(
        drive
            .run(() -> drive.setRobotSpeeds(pipeline.getChassisSpeeds()))
            .finallyDo(drive::stop)
            .withName("Pipeline Drive"));

    DriverDashboard.currentDriveModeName =
        () -> {
          Command current = drive.getCurrentCommand();
          if (current == drive.getDefaultCommand()) {
            String layers = String.join(" + ", pipeline.getActiveLayers());
            return "[" + layers + "]";
          } else if (current != null) {
            return current.getName();
          }
          return "Idle";
        };

    // Toggle robot relative mode, used as backup if gyro fails
    xbox.y()
        .toggleOnTrue(
            pipeline.activateLayer(
                input -> input.fieldRelativeDisabled().addLabel("Robot Relative")));

    // Secondary drive command, right stick will be used to control target angular position instead
    // of angular velocity
    xbox.rightBumper()
        .whileTrue(
            pipeline.activateLayer(
                input ->
                    input
                        .headingStick(() -> -xbox.getRightY(), () -> -xbox.getRightX())
                        .addLabel("Heading Controlled")));

    // Face zero point of the field (testing)
    xbox.a()
        .whileTrue(
            pipeline.activateLayer(
                input -> input.facingPoint(Translation2d.kZero).addLabel("Face Point")));

    // Slow mode, reduce translation and rotation speeds for fine control
    xbox.leftBumper()
        .whileTrue(
            pipeline.activateLayer(
                input ->
                    input
                        .translationCoefficient(0.3)
                        .rotationCoefficient(0.1)
                        .addLabel("Slow Mode")));

    // Cause the robot to resist movement by forming an X shape with the swerve modules
    // Helps prevent getting pushed around
    xbox.x()
        .whileTrue(drive.run(drive::stopUsingBrakeArrangement).withName("RESIST Movement With X"));

    // Stop the robot and cancel any running commands
    xbox.b()
        .or(RobotModeTriggers.disabled())
        .onTrue(drive.runOnce(drive::stop).withName("CANCEL and Stop"));

    xbox.b()
        .debounce(1)
        .onTrue(rumbleController(xbox, 0.3).withTimeout(0.25))
        .whileTrue(drive.run(drive::stopUsingForwardArrangement).withName("Stop and Orient"));

    // Reset the gyro heading
    xbox.start()
        .debounce(0.3)
        .onTrue(
            drive
                .runOnce(
                    () ->
                        drive.resetPose(
                            new Pose2d(drive.getRobotPose().getTranslation(), Rotation2d.kZero)))
                .andThen(rumbleController(xbox, 0.3).withTimeout(0.25))
                .ignoringDisable(true)
                .withName("Reset Gyro Heading"));

    // Configure the driving dpad
    configureDrivingDpad(xbox, pipeline, 1, true);
  }

  private void configureDrivingDpad(
      CommandXboxController xbox,
      DriveInputPipeline pipeline,
      double strafeSpeed,
      boolean includeDiagonals) {
    for (int pov = 0; pov < 360; pov += includeDiagonals ? 45 : 90) {
      Rotation2d rotation = Rotation2d.fromDegrees(-pov);
      Translation2d translation = new Translation2d(strafeSpeed, rotation);
      String name = "DPad Drive " + pov;
      Command activateLayer =
          pipeline.activateLayer(
              input ->
                  input
                      .translation(() -> translation)
                      .fieldRelativeDisabled()
                      .rotationCoefficient(0.3)
                      .addLabel(name));
      xbox.pov(pov).whileTrue(activateLayer);
    }
  }

  private void configureOperatorControllerBindings(CommandXboxController xbox) {}

  private Command rumbleController(CommandXboxController controller, double rumbleIntensity) {
    return Commands.startEnd(
            () -> controller.setRumble(RumbleType.kBothRumble, rumbleIntensity),
            () -> controller.setRumble(RumbleType.kBothRumble, 0))
        .withName("RumbleController");
  }

  private Command rumbleControllers(double rumbleIntensity) {
    return Commands.parallel(
        rumbleController(driverController, rumbleIntensity),
        rumbleController(operatorController, rumbleIntensity));
  }

  private void configureAlertTriggers() {
    // Endgame alert triggers
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= 20)
        .onTrue(rumbleControllers(0.5).withTimeout(0.5));

    RobotModeTriggers.teleop()
        .and(RobotBase::isReal)
        .onChange(rumbleControllers(0.2).withTimeout(0.2));

    Trigger isMatch = new Trigger(() -> DriverStation.getMatchTime() != -1);

    RobotModeTriggers.teleop()
        .and(isMatch)
        .onTrue(Commands.runOnce(() -> Elastic.selectTab("Teleoperated")));

    RobotModeTriggers.autonomous()
        .and(isMatch)
        .onTrue(Commands.runOnce(() -> Elastic.selectTab("Autonomous")));
  }

  private void registerNamedCommands() {
    // Set up named commands for path planner auto
  }

  private void configureAutos(LoggedDashboardChooser<Command> dashboardChooser) {

    // Path planner Autos
    // https://pathplanner.dev/gui-editing-paths-and-autos.html#autos

    // Choreo Autos
    // https://pathplanner.dev/pplib-choreo-interop.html#load-choreo-trajectory-as-a-pathplannerpath

    if (Constants.RUNNING_TEST_PLANS) {
      dashboardChooser.addOption(
          "[Characterization] Drive Feed Forward",
          DriveCommands.feedforwardCharacterization(drive));
      dashboardChooser.addOption(
          "[Characterization] Drive Wheel Radius",
          DriveCommands.wheelRadiusCharacterization(drive));
    }
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
