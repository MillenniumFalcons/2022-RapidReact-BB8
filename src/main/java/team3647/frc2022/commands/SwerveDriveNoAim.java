// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import team3647.frc2022.constants.SwerveDriveConstants;
import team3647.frc2022.subsystems.SwerveDrive;

public class SwerveDriveNoAim extends CommandBase {
    private final SwerveDrive swerve;

    private final DoubleSupplier xSpeedFunction;
    private final DoubleSupplier ySpeedFunction;
    private final DoubleSupplier turnSpeedFunction;
    private final BooleanSupplier getIsFieldOriented;

    private double rotation;
    private Translation2d translation;

    private final double translateMultiplier = 1.0; // 0.1;
    private final double rotationMultiplier = 1.0; // 0.1;

    /** Creates a new SwerveDriveTeleopBaseFalcon. */
    public SwerveDriveNoAim(
            SwerveDrive swerve,
            DoubleSupplier xSpeedFunction,
            DoubleSupplier ySpeedFunction,
            DoubleSupplier turnSpeedFunction,
            BooleanSupplier getIsFieldOriented) {
        this.swerve = swerve;
        this.xSpeedFunction = xSpeedFunction;
        this.ySpeedFunction = ySpeedFunction;
        this.turnSpeedFunction = turnSpeedFunction;
        this.getIsFieldOriented = getIsFieldOriented;
        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xComponent = ySpeedFunction.getAsDouble() * translateMultiplier;
        double yComponent = -xSpeedFunction.getAsDouble() * translateMultiplier;
        translation =
                new Translation2d(xComponent, yComponent)
                        .times(SwerveDriveConstants.kDrivePossibleMaxSpeedMPS);
        rotation =
                -turnSpeedFunction.getAsDouble()
                        * SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec
                        * rotationMultiplier;

        swerve.drive(translation, rotation, getIsFieldOriented.getAsBoolean(), false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
