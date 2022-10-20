package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3647.frc2022.subsystems.Wrist;

public class WristCommands {
    private final Wrist wrist;

    public WristCommands(Wrist wrist) {
        this.wrist = wrist;
    }

    public Command deploy() {
        return new InstantCommand(wrist::extend);
    }

    public Command retract() {
        return new InstantCommand(wrist::retract);
    }
}
