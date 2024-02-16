package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class Climb extends Command {
    private CANSparkMax leftMotor = new CANSparkMax(57, MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(41, MotorType.kBrushless);
    private XboxController XBOX;
    private double zCommand = 0;
    

    public Climb(XboxController XBOX) {
        this.XBOX = XBOX;




    }

    @Override
    public void execute() {
        zCommand = XBOX.getLeftY();
        zCommand = (Math.abs(zCommand) < Constants.stickDeadband) ? 0 : zCommand;
        zCommand = Math.copySign(zCommand * zCommand / 2, Math.signum(zCommand));
        leftMotor.set(zCommand);
        rightMotor.follow(leftMotor);


        
    }

    
}
