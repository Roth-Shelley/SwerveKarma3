package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ReverseIntakeCommand extends Command {

    private final Intake intake;

    
    public ReverseIntakeCommand(Intake intakeSubystem) {
        this.intake = intakeSubystem;
        addRequirements(intakeSubystem);
    }

    public void execute() {
        //write code that makes the motor move at -0.5 speed when a button on the game controller is pressed

        intake.setIntakeMotorSpeed(-0.5);
    }

    public void end(boolean interrupted) {
        
    }


}


