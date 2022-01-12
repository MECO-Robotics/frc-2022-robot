// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.BallCollectionSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Spins the intake roller to pull in balls */
public class Outtake extends CommandBase {

  private final BallCollectionSubsystem ballCollector;

  /**
   * Creates a new Command.
   *
   * @param ballCollectionSubsystem The subsystem used by this command.
   */
  public Outtake(BallCollectionSubsystem ballCollectionSubsystem) {
    ballCollector = ballCollectionSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ballCollectionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballCollector.moveIntake(0.0, 0.7);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballCollector.moveIntake(0.0, 0.0);
  }

  // Returns true when the command should end. (this command never finishes)
  @Override
  public boolean isFinished() {
    return false;
  }
}