package frc.math;

import edu.wpi.first.math.geometry.Pose2d;

public class LimelightTargetCalculator {
   boolean isBlue ;
  
 


    public LimelightTargetCalculator(boolean isBlue) {
        this.isBlue = isBlue;


    }


    public double calculateCoefficient(Pose2d pose) {
        Pose2d pose2 = CoordinateSystems.FieldMiddle_FieldBottomLeft(pose);
        double poseX = pose2.getX();
        double poseY = pose2.getY();
        //sees if robot is facing left or right
        double horizontalDotProduct = 0.0;
        double coefficentAprilTags = 0;
        
        if (Math.PI / 2 < pose.getRotation().getRadians() && pose.getRotation().getRadians()< 3 * Math.PI / 2) {
            horizontalDotProduct =  isBlue ? Math.cos(pose.getRotation().getRadians()) : -Math.cos(pose.getRotation().getRadians());

//same as a dot product
        }

        else {
             horizontalDotProduct =  isBlue ? Math.cos(pose.getRotation().getRadians()) : -Math.cos(pose.getRotation().getRadians());

        }



        if (horizontalDotProduct > 0) {
            if (poseX < 1.14) { 
                coefficentAprilTags = 1;


            }

            else if (poseX < 2.312) {
                coefficentAprilTags = 0.4;

            }
            else if (poseX < 2.312 + (3.4591 - 1.5509)) {

                coefficentAprilTags = 1;
            }

            else {
                coefficentAprilTags = 2;

            }


        }

        else {

            if (poseX < 1.14) { 
                coefficentAprilTags = 2.5;


            }

            else if (poseX < 2.312) {
                coefficentAprilTags = 1.8;

            }
            else if (poseX < 2.312 + (3.4591 - 1.5509)) {

                coefficentAprilTags = 1;
            }

            else {
                coefficentAprilTags = 1.25;

            }



        }


        return 0.1 * (3.5 - coefficentAprilTags * horizontalDotProduct);


    }
}