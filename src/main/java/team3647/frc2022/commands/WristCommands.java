package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import team3647.frc2022.constants.WristIntakeConstants;
import team3647.frc2022.subsystems.Wrist;

public class WristCommands {
    private final Wrist wrist;

    public WristCommands(Wrist wrist) {
        this.wrist = wrist;
    }

    public Command deploy() {
        return new InstantCommand(wrist::extend);
    }

    public Command deployLow() {
        return new InstantCommand(wrist::extendLower);
    }

    public Command retract() {
        return new InstantCommand(wrist::retract);
    }

    public Command holdPositionAtCall() {
        return new Command() {
            double degreeAtStart = WristIntakeConstants.zeroDeg;

            @Override
            public void initialize() {
                degreeAtStart = wrist.getAngle();
            }

            @Override
            public void execute() {
                wrist.setAngleMotionMagic(degreeAtStart);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(wrist);
            }
        };
    }
}
