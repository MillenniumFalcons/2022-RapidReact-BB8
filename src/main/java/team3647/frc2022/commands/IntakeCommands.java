package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.function.DoubleSupplier;
import team3647.frc2022.subsystems.WristIntake;

public class IntakeCommands {
    private final WristIntake intake;

    public IntakeCommands(WristIntake intake) {
        this.intake = intake;
    }

    public Command deploy() {
        return new InstantCommand(intake::extend);
    }

    // public Command retract() {
    //     // return new InstantCommand(intake::retract);
    // }

    public Command openLoopAndStop(double percentOut) {
        return new FunctionalCommand(
                () -> {},
                () -> intake.setOpenloop(percentOut),
                interrupted -> {
                    intake.setOpenloop(0);
                },
                () -> false,
                intake);
    }

    public Command runClosedLoop(DoubleSupplier surfaceVel) {
        return new FunctionalCommand(
                () -> {},
                () -> intake.setSurfaceVelocity(surfaceVel.getAsDouble()),
                interrupted -> {
                    intake.setOpenloop(0);
                    // intake.retract();
                },
                () -> false,
                intake);
    }

    public Command deployAndIntakeWith(Command intakingCommand) {
        return deploy().andThen(intakingCommand);
    }
}
