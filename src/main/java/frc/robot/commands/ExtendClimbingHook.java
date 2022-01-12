// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClimbingArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Raise the climbing hook. */
public class ExtendClimbingHook extends CommandBase {

  private final ClimbingArmSubsystem climbingSubsystem;

  /**
   * Creates a new Command.
   *
   * @param ballCollectionSubsystem The subsystem used by this command.
   */
  public ExtendClimbingHook(ClimbingArmSubsystem climbingSubsystem) {
    this.climbingSubsystem = climbingSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbingSubsystem.moveArm(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbingSubsystem.moveArm(0.0);
  }

  // Returns true when the command should end. (this command never finishes)
  @Override
  public boolean isFinished() {
    return false;
  }
}