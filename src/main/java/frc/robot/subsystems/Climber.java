package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase {
    public double distance = 0;
    
    CANSparkMax climbLeftMotor = new CANSparkMax(ClimbConstants.climbLeftMotorID, MotorType.kBrushless);
    CANSparkMax climbRightMotor = new CANSparkMax(ClimbConstants.climbRightMotorID, MotorType.kBrushless);

    RelativeEncoder climbLeftEncoder = climbLeftMotor.getEncoder();
    RelativeEncoder climbRightEncoder = climbRightMotor.getEncoder();




    public Climber() {
        climbLeftMotor.restoreFactoryDefaults();
        climbRightMotor.restoreFactoryDefaults();


        
        climbLeftEncoder.setPosition(0);
        climbRightEncoder.setPosition(0);
        
    }
    
}
