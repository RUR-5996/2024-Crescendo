package frc.robot.Subsystems;

import frc.robot.Subsystems.LimelightHelpers;

public class LimeLight {

    public static double[] getRelativePos() {
        double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
        double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
        double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)

        double tagID = LimelightHelpers.getFiducialID(null);

        double[] posOffset = {tx, ty, ta, tagID};

        return posOffset;
    } 
}
