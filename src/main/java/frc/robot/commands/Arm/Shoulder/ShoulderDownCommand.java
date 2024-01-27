// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.Shoulder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderDownCommand extends Command {
  private ShoulderSubsystem shoulderSubsystem;
  public ShoulderDownCommand(ShoulderSubsystem m_ShoulderSubsystem) {
    shoulderSubsystem = m_ShoulderSubsystem;
    addRequirements(shoulderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoulderSubsystem.shoulderDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoulderSubsystem.shoulderStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
