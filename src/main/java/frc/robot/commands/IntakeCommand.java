package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class IntakeCommand extends Command {

    private final Intake intake;

    public IntakeCommand(Intake intakeSubystem) {
        this.intake = intakeSubystem;
        addRequirements(intakeSubystem);
    }

    public void execute() {
        //write code that makes the motor move with a game controller speed
        intake.setIntakeMotorSpeed(0.5);

    }

    public void end(boolean interrupted) {
    }

}
