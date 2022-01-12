// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.BallCollectionSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Raises the arm. */
public class LowerBallCollectionArm extends CommandBase {

  private final BallCollectionSubsystem ballCollector;

  /**
   * Creates a new ExampleCommand.
   *
   * @param ballCollectionSubsystem The subsystem used by this command.
   */
  public LowerBallCollectionArm(BallCollectionSubsystem ballCollectionSubsystem) {
    ballCollector = ballCollectionSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ballCollectionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Trigger moving up
    ballCollector.moveArm(false, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Just keep checking the arm movement, without the up button pressed
    ballCollector.moveArm(false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end. (this command never finishes)
  @Override
  public boolean isFinished() {
    // If the motor speed is zero, we're done. This is checked after
    // the initialize() and execute() methods, which will set the arm
    // speed to non-zero.
   
    if (ballCollector.getCurrentArmMotorSpeed() == 0.0){
      System.out.println("finished");
      return true;
      
    }

    else {
      return false;
    }
    
  }
 
}