// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class HalfSpeed extends Command {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private SwerveSubsystem s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    /**
     * Driver control
     */
    public HalfSpeed(SwerveSubsystem s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        /*sets the axis to the controller sticks*/
        double yAxis = -controller.getRawAxis(translationAxis)*Constants.Swerve.rotationMultiplier;
        double xAxis = -controller.getRawAxis(strafeAxis)*Constants.Swerve.rotationMultiplier;
        double rAxis = -controller.getRawAxis(rotationAxis)*Constants.Swerve.rotationMultiplier;
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.slowstickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.slowstickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.slowstickDeadband) ? 0 : rAxis;

        translation = new Translation2d(yAxis*(0.25), xAxis*(0.25)).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity * 0.2;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}