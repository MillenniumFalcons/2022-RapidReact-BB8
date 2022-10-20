// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands.turret;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import team3647.frc2022.subsystems.Turret;
import team3647.lib.vision.AimingParameters;

public class AimTurret extends CommandBase {
    private final Turret turret;
    private final Supplier<AimingParameters> aimingParameters;
    private final Supplier<Twist2d> robotVelocity;
    /** Creates a new AimTurret. */
    public AimTurret(
            Turret turret,
            Supplier<AimingParameters> aimingParameters,
            Supplier<Twist2d> robotVelocity) {
        this.aimingParameters = aimingParameters;
        this.turret = turret;
        this.robotVelocity = robotVelocity;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var aimingParams = aimingParameters.get();
        if (aimingParams == null) {
            return;
        }
        if (aimingParams.getRangeMeters() > 8) {
            return;
        }
        double turretSetpoint =
                turret.getAngle() + aimingParams.getTurretAngleToTarget().getDegrees();
        Twist2d velocity = robotVelocity.get();
        // Angular velocity component from tangential robot motion about the goal.
        double tangential_component =
                aimingParams.getRobotToTargetTransform().getRotation().getSin()
                        * velocity.dx
                        / aimingParams.getRangeMeters();
        double angular_component = Units.radiansToDegrees(velocity.dtheta);
        // Add (opposite) of tangential velocity about goal + angular velocity in local frame.
        var ff = -(angular_component + tangential_component);
        turret.setAngle(turretSetpoint, ff);
        // System.out.println("Turret: " + turretSetpoint + ", velocity: " + ff);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        turret.setOpenloop(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
