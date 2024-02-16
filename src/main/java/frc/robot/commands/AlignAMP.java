package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PipelineConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

public class AlignAMP extends Command {
    double visionAngle;
    private final Swerve swervy;
    private final VisionSubsystem vision;
    private final PIDController rotationPID;
    private TeleopSwerve swervo;

    public AlignAMP(Swerve swerve, VisionSubsystem vision, PIDController rotationPID) {
        this.swervy = swerve;
        this.vision = vision;
        this.rotationPID  = rotationPID;
        addRequirements(vision);

        if (vision.isBlue) {
            vision.setPipelineLL3(PipelineConstants.FiducialTargets);

        }

        else {
            vision.setPipelineLL3(PipelineConstants.FiducialTargets);
        }
    }

    public void execute() {
        if (vision.getFiducialDetection) {
            
        }
        else {
        }
    }
    
}
