// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.commands.Arm.Elevator.ElevatorGoToPosition;
import frc.robot.commands.Arm.Shoulder.GoToPosShoulder;
import frc.robot.commands.Arm.Wrist.GoToPositionWrist;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmTop extends SequentialCommandGroup {
  public ArmTop(ElevatorSubsystem elevator, ShoulderSubsystem shoulder, WristSubsystem wrist) {
    GlobalVariables.elevatordesiredpos = 122.64*0.6;
    GlobalVariables.shoulderdesiredpos = Constants.Arm.Shoulder.targetPosition7-Constants.Arm.Shoulder.shoulderOffset;
    GlobalVariables.wristdesiredpos = 32.5;
    addCommands(
      new ParallelCommandGroup(
        new ElevatorGoToPosition(elevator, 122.64 * .6),
        new GoToPosShoulder(shoulder, Constants.Arm.Shoulder.targetPosition7-Constants.Arm.Shoulder.shoulderOffset),
        new GoToPositionWrist(wrist, 32.275) //0.579
      ).withTimeout(3)
    );
  }
}
