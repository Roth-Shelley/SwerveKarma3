package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RampConstants;



public class ShooterAndRamp extends SubsystemBase {

     CANSparkMax shooterRightMotor = new CANSparkMax(RampConstants.rampMotorID, MotorType.kBrushless);
     CANSparkMax shooterLeftMotor = new CANSparkMax(RampConstants.rampMotorID, MotorType.kBrushless);
     CANSparkMax rampLeftMotor = new CANSparkMax(RampConstants.rampMotorID, MotorType.kBrushless);
     CANSparkMax rampRightMotor = new CANSparkMax(RampConstants.rampMotorID, MotorType.kBrushless);

    //  SparkMaxPIDController 
    RelativeEncoder rampLeftEncoder = rampLeftMotor.getEncoder();
    RelativeEncoder rampRightEncoder = rampRightMotor.getEncoder();

    RelativeEncoder shooterLeftEncoder = shooterLeftMotor.getEncoder();
    RelativeEncoder shooterRightEncoder = shooterRightMotor.getEncoder();




    public ShooterAndRamp() {
        rampLeftMotor.restoreFactoryDefaults();
        rampRightMotor.restoreFactoryDefaults();

       
    
        rampLeftEncoder.setPosition(0);
        rampRightEncoder.setPosition(0);


        // intakeMotor.follow(intakeMotor);
        // rampMotor.follow(rampMotor);

    }
    public enum States {
        HOME,
        SHOOTING,
        AMPING,
        HANDOFF_ARM,
        

    }
    


    public void setRampMotorSpeed(double speed) {
        rampLeftMotor.set(speed);
        rampRightMotor.set(-speed);
    }



    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}