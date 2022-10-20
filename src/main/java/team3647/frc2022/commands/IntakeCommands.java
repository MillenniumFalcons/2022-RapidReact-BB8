package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.function.DoubleSupplier;
import team3647.frc2022.subsystems.Intake;

public class IntakeCommands {
    private final Intake intake;

    public IntakeCommands(Intake intake) {
        this.intake = intake;
    }

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
}
