package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.math.CoordinateSystems;
import frc.robot.Constants.PipelineConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.*;


public class AlignAMP extends Command {
    double visionAngle;
    private final Swerve swervy;
    private Pose2d targetPose;
    private Pose2d myPose;
   

    public AlignAMP(Swerve swerve, VisionSubsystem vision) {
        this.swervy = swerve;
        addRequirements(swerve);

        if (vision.isBlue) {
        myPose = CoordinateSystems.LeftSide_RobToField(swervy.getPose());

        targetPose = new Pose2d(new Translation2d(-6.3627, 4.105656 - 0.55), new Rotation2d(Math.PI/2));
        }

        else {
        targetPose = new Pose2d(new Translation2d(6.3627, 4.105656 - 0.55), new Rotation2d(Math.PI/2));
        myPose = CoordinateSystems.RightSide_RobToField(swervy.getPose());
        }

        PathConstraints constraints = new PathConstraints(2, 2, Units.degreesToRadians(360), Units.degreesToRadians(360));
        

        
    }

    public void execute() {




    }
    
}



