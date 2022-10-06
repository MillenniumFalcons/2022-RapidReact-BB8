package team3647.frc2022.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2022.subsystems.SwerveDrive;

public class TeleopAim extends CommandBase {
    private static SwerveDrive m_swerve;
    private static Translation2d target;

    public TeleopAim(Translation2d target) {
        // this.m_swerve = swerve;
        // this.target = target;
        // addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // System.out.print(angleBetweenPoints(m_swerve.getPose().getTranslation(), target));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
