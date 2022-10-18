package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2022.subsystems.WristIntake;

public class DeployIntake extends CommandBase {
    private WristIntake wristIntake;

    public DeployIntake(WristIntake wristIntake) {
        this.wristIntake = wristIntake;
        addRequirements(wristIntake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        wristIntake.extend();
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
