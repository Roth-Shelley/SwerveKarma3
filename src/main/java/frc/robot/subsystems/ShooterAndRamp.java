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

    public String currentState; 




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

    public States whichStateAreWeCurrentlyIn() {
        if(currentState.equals("HOME")) {
            return States.HOME;
        }
        else if(currentState.equals("SHOOTING")) {
            return States.SHOOTING;
        }
        else if(currentState.equals("AMPING")) {
            return States.AMPING;
        }
        else if(currentState.equals("HANDOFF_ARM")) {
            return States.HANDOFF_ARM;
        }
        else {
            return States.HOME;
        }
        }



    public void setRampMotorSpeed(double speed) {
        rampLeftMotor.set(speed);
        rampRightMotor.set(-speed);
    }

    public void executeRampState(String state) {
        switch(currentState) {
            case "HOME":
                setRampMotorSpeed(0);
                break;
            case "SHOOTING":
                setRampMotorSpeed(0.5);
                break;
            case "AMPING":
                setRampMotorSpeed(0.5);
                break;
            case "HANDOFF_ARM":
                setRampMotorSpeed(0.5);
                break;
        }
    }

 public void executeRampState(States state) {
        String currentState = state.toString();
        switch(currentState) {
            case "HOME":
                setRampMotorSpeed(0);
                break;
            case "SHOOTING":
                setRampMotorSpeed(0.5);
                break;
            case "AMPING":
                setRampMotorSpeed(0.5);
                break;
            case "HANDOFF_ARM":
                setRampMotorSpeed(0.5);
                break;
        }
    }




    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}