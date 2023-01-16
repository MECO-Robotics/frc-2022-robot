// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.trajectory.constraint.MecanumDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.auto.AutoShootCollectRightShoot;
import frc.robot.commands.auto.DriveBackwardsAuto;
import frc.robot.commands.auto.HalfBall;
import frc.robot.commands.auto.RamShoot;
//import frc.robot.commands.auto.Roboticon;
import frc.robot.commands.auto.SpinShoot;
import frc.robot.commands.auto.SpinShootBombSquad;
import frc.robot.commands.auto.TwoBallAuto;
import frc.robot.commands.auto.TwoBallAuto_OneAtATime;
import frc.robot.commands.auto.TwoBallAuto_TwiceSameTime;
import frc.robot.commands.auto.TwoBallAuto_TwiceSameTimeV3;
import frc.robot.commands.cargo.Collect;
import frc.robot.commands.cargo.Intake;
import frc.robot.commands.cargo.LowerCargoElbow;
import frc.robot.commands.cargo.LowerCargoWrist;
import frc.robot.commands.cargo.Outtake;
import frc.robot.commands.cargo.RaiseCargoElbow;
import frc.robot.commands.cargo.RaiseCargoWrist;
import frc.robot.commands.cargo.Shoot;
import frc.robot.commands.cargo.Stow;
import frc.robot.commands.climb.TeleopRotatingArmPneumaticIn;
import frc.robot.commands.climb.TeleopRotatingArmPneumaticOff;
import frc.robot.commands.climb.TeleopRotatingArmPneumaticOut;
import frc.robot.commands.climb.Climb;
import frc.robot.commands.climb.ResetWinchEncoders;
import frc.robot.commands.climb.RotatingArmRaiseFullOpenGrip;
import frc.robot.commands.climb.CopilotJoysticksControlWinches;
import frc.robot.commands.demo.MoveOctagon;
import frc.robot.commands.drive.Energize;
import frc.robot.commands.drive.Stop;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.commands.drive.ToggleDirection;
import frc.robot.commands.lights.TurnBlueBothOn;
import frc.robot.commands.lights.TurnBlueLeftOnly;
import frc.robot.commands.lights.TurnBlueOffBoth;
import frc.robot.commands.lights.TurnGreenBothOff;
import frc.robot.commands.lights.TurnGreenBothOn;
import frc.robot.commands.lights.TurnPurpleRightOff;
import frc.robot.commands.lights.TurnPurpleRightOn;
import frc.robot.commands.lights.TurnRedBothOn;
import frc.robot.commands.lights.TurnRedOffBoth;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.PowerHub;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final CargoSubsystem cargoSubsystem = new CargoSubsystem();
  private final ClimbingSubsystem climbingSubsystem = new ClimbingSubsystem();
  private final ControllerSubsystem controllerSubsystem = new ControllerSubsystem();
  private final LightSubsystem lightSubsystem = new LightSubsystem();
  private final PowerHub powerHub = new PowerHub();

  private final Map<String, Command> autoCommands = new HashMap<>();
  private final SendableChooser<String> autoCommandChoice = new SendableChooser<String>();

  private final SendableChooser<DriveMode> driveMode = new SendableChooser<DriveMode>();

  public enum DriveMode {
    SplitArcade,
    Tank,
    Joystick
  }

  /**
   * The container for the robot. Contains subsystems, I/O devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // ------------------------------------------------------------------------
    //
    // PLACE ALL AUTONOMOUS COMMANDS HERE
    //

    // Create a mapping of name to command object for every autonomous command
    // By using the class name as the name, it will be easy to remember which goes
    // with which.
    autoCommands.put("MoveOctagon", new MoveOctagon(driveSubsystem));
    // autoCommands.put("BallAuto", new BallAuto(driveSubsystem, cargoSubsystem));
    //autoCommands.put("AutoShootCollectRightShoot", new AutoShootCollectRightShoot(driveSubsystem));
    autoCommands.put("DriveBack", new DriveBackwardsAuto(driveSubsystem));
    //autoCommands.put("Ram Shot", new RamShoot(driveSubsystem, cargoSubsystem));
    autoCommands.put("1 Ball ", new SpinShoot(driveSubsystem, cargoSubsystem));
    autoCommands.put("HalfBall", new HalfBall(driveSubsystem, cargoSubsystem));
    //autoCommands.put("BombSquad", new SpinShootBombSquad(driveSubsystem, cargoSubsystem));
    autoCommands.put("2 Ball v1", new TwoBallAuto_OneAtATime(driveSubsystem, cargoSubsystem));
    autoCommands.put("2 Ball v2", new TwoBallAuto_TwiceSameTime(driveSubsystem, cargoSubsystem));
    autoCommands.put("2 ball v3", new TwoBallAuto_TwiceSameTimeV3(driveSubsystem, cargoSubsystem));
    //autoCommands.put("roboticon", new Roboticon(driveSubsystem, cargoSubsystem));
    //
    //
    // ------------------------------------------------------------------------
    
    for (String choiceName : autoCommands.keySet()) {
      autoCommandChoice.addOption(choiceName, choiceName);
    }
    autoCommandChoice.setDefaultOption("DriveBack", "DriveBack");
    SmartDashboard.putData("Autonomous Mode", autoCommandChoice);

    driveMode.setDefaultOption("Split Arcade", DriveMode.SplitArcade);
    driveMode.addOption("Tank", DriveMode.Tank);
    driveMode.addOption("Joystick", DriveMode.Joystick);
    SmartDashboard.putData("Drive mode", driveMode);

    SmartDashboard.putData("PNEUM IN", new TeleopRotatingArmPneumaticIn(climbingSubsystem));
    SmartDashboard.putData("PNEUM OUT", new TeleopRotatingArmPneumaticOut(climbingSubsystem));
    SmartDashboard.putData("PNEUM OFF", new TeleopRotatingArmPneumaticOff(climbingSubsystem));
    SmartDashboard.putData("ElbowDown", new LowerCargoElbow(cargoSubsystem));
    SmartDashboard.putData("ElbowUp", new RaiseCargoElbow(cargoSubsystem));
    SmartDashboard.putData("WristDown", new LowerCargoWrist(cargoSubsystem));
    SmartDashboard.putData("WristUp", new RaiseCargoWrist(cargoSubsystem));
    SmartDashboard.putData("RESET CLIMB ENC", new ResetWinchEncoders(climbingSubsystem));
    SmartDashboard.putData("Red Lights On", new TurnRedBothOn(lightSubsystem));
    SmartDashboard.putData("Red Lights Off", new TurnRedOffBoth(lightSubsystem));
    SmartDashboard.putData("Blue Lights On", new TurnBlueBothOn(lightSubsystem));
    SmartDashboard.putData("Blue Lights Off", new TurnBlueOffBoth(lightSubsystem));
    SmartDashboard.putData("Green Lights On", new TurnGreenBothOn(lightSubsystem));
    SmartDashboard.putData("Green Ligths Off", new TurnGreenBothOff(lightSubsystem));
    SmartDashboard.putData("Purple Lights On", new TurnPurpleRightOn(lightSubsystem));
    SmartDashboard.putData("Purple Lights Off", new TurnPurpleRightOff(lightSubsystem));
    SmartDashboard.putData("Blue Lights Left On", new TurnBlueLeftOnly(lightSubsystem));

    // Set default commands
    driveSubsystem.setDefaultCommand(new Stop(driveSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // This is an example of button bindings using the new approach. The rest
    // of the button bindings are actually done in the ControllerSubsystem class.

    configureTeleopDriveButtonBindings();

    configureTeleopCargoButtonBindings();

    configureTeleopClimbButtonBindings();
  }

  /**
   * Setup the buttons for controlling climbing during teleop
   */
  private void configureTeleopClimbButtonBindings() {

    // NATE: bind buttons for teleop climb
    XboxController copilot = controllerSubsystem.getCopilotController();
    JoystickButton leftBumper = new JoystickButton(copilot, XboxController.Button.kLeftBumper.value);
    JoystickButton righBumper = new JoystickButton(copilot, XboxController.Button.kRightBumper.value);
    JoystickButton xButton = new JoystickButton(copilot, XboxController.Button.kX.value);
    JoystickButton yButton = new JoystickButton(copilot, XboxController.Button.kY.value);
    JoystickButton bButton = new JoystickButton(copilot, XboxController.Button.kB.value);
    

    XboxController pilot = controllerSubsystem.getPilotController();
    JoystickButton xButtonpilot = new JoystickButton(pilot, XboxController.Button.kX.value);
    JoystickButton yButtonpilot = new JoystickButton(pilot, XboxController.Button.kY.value);
    xButtonpilot.whenPressed(new TeleopRotatingArmPneumaticIn(climbingSubsystem));
    yButtonpilot.whenPressed(new TeleopRotatingArmPneumaticOut(climbingSubsystem));

    leftBumper.whenPressed(new TeleopRotatingArmPneumaticIn(climbingSubsystem), false);
    righBumper.whenPressed(new TeleopRotatingArmPneumaticOut(climbingSubsystem), false);
    xButton.whenPressed(new TeleopRotatingArmPneumaticOff(climbingSubsystem), false);
    bButton.whenPressed(new Climb(climbingSubsystem), false);
    yButton.whenPressed(new TeleopRotatingArmPneumaticOut(climbingSubsystem), false);
  }

  /**
   * Setup the buttons for collecting and shooting cargo
   */
  private void configureTeleopCargoButtonBindings() {
    XboxController copilot = controllerSubsystem.getCopilotController();
    XboxController pilot = controllerSubsystem.getPilotController();

    JoystickButton pilotLeftBumper = new JoystickButton(pilot, XboxController.Button.kLeftBumper.value);
    JoystickButton pilotRightBumper = new JoystickButton(pilot, XboxController.Button.kRightBumper.value);

    pilotRightBumper.whenHeld(new Intake(cargoSubsystem), true);
    pilotLeftBumper.whenHeld(new Outtake(cargoSubsystem), true);

    POVButton copilotDpadUp = new POVButton(copilot, 0);
    POVButton copilotDpadDown = new POVButton(copilot, 180);
    POVButton copilotDpadRight = new POVButton(copilot, 90);

    POVButton pilotDpadUp = new POVButton(pilot, 0);
    POVButton pilotDpadDown = new POVButton(pilot, 180);
    POVButton pilotDpadRight = new POVButton(pilot, 90);
    POVButton pilotStartButton = new POVButton(pilot, 0);

    // POVButton copilotDpadLeft = new POVButton(copilot, 270);

    pilotStartButton.whenHeld(new Energize(driveSubsystem), true);
    pilotDpadUp.whenPressed(new Shoot(cargoSubsystem), true);
    pilotDpadDown.whenPressed(new Collect(cargoSubsystem), true);
    pilotDpadRight.whenPressed(new Stow(cargoSubsystem), true);

    copilotDpadUp.whenPressed(new Shoot(cargoSubsystem), true);
    copilotDpadDown.whenPressed(new Collect(cargoSubsystem), true);
    copilotDpadRight.whenPressed(new Stow(cargoSubsystem), true);
  }

  /**
   * Setup the buttons for teleop drive.
   */
  private void configureTeleopDriveButtonBindings() {
    JoystickButton modeButton = new JoystickButton(controllerSubsystem.getPilotController(),
        XboxController.Button.kStart.value);
    modeButton.whenPressed(new ToggleDirection(controllerSubsystem), false);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command command = null;

    if (autoCommandChoice != null && autoCommandChoice.getSelected() != null) {
      command = autoCommands.get(autoCommandChoice.getSelected());
    }

    return command;
  }

  public Command getTeleopDriveCommand() {

    return new ParallelCommandGroup(
        new TeleopDrive(driveSubsystem, controllerSubsystem, driveMode.getSelected()),
        new CopilotJoysticksControlWinches(climbingSubsystem, controllerSubsystem));
  }

  public DriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }
}
