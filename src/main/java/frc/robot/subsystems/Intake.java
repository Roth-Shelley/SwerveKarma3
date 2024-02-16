package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.Constants.IntakeConstants;


public class Intake extends SubsystemBase {


    //CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);

   // RelativeEncoder intakeEncoder = intakeMotor.getEncoder();




    public Intake() {
       // intakeMotor.restoreFactoryDefaults();

        
        //intakeEncoder.setPosition(0);

        // intakeMotor.follow(intakeMotor);
        // rampMotor.follow(rampMotor);

    }

    public void setIntakeMotorSpeed(double speed) {
        //intakeMotor.set(speed);
    }
    


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}