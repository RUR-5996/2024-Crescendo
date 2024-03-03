package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



class LimelightNetwork{
    NetworkTable limelightTable; // might need to change table

    NetworkTableEntry tX;
    NetworkTableEntry tY;
    NetworkTableEntry tA;
    NetworkTableEntry tV;
    public NetworkTableEntry pipeline;

    public double X = 0.0;
    public double Y = 0.0;
    public double A = 0.0;
    public boolean V = false;

    public LimelightNetwork(String key){
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

        tX = limelightTable.getEntry("tx");
        tY = limelightTable.getEntry("ty");
        tA = limelightTable.getEntry("ta");
        tV = limelightTable.getEntry("tv");
        pipeline = limelightTable.getEntry("pipeline");

        pipeline.setDouble(0);
    }

    public void update(){
        X = tX.getDouble(0.0);
        Y = tY.getDouble(0.0);
        A = tA.getDouble(0.0);
        V = tV.getDouble(0.0) == 1.0;
    }
    
}


public class Limelight{
    
    static LimelightNetwork Limelight1;

    // Limelight 
    static enum LimelightMode {
        AprilTag, 
        Color
    }
    // PUBLIC
    public static LimelightMode mode = LimelightMode.Color;

    public static void init(){
        Limelight1 = new LimelightNetwork("limelight");
    }

    public static void periodic(){
        if(mode == LimelightMode.Color)
            Limelight1.pipeline.setNumber(0);
        else if(mode == LimelightMode.AprilTag)
            Limelight1.pipeline.setNumber(1);
        runLimelight();

        report();
    }

    public static void report() {
        SmartDashboard.putNumber("LimelightMode", Limelight1.pipeline.getDouble(0 ));
        SmartDashboard.putNumber("LimelightX", Limelight1.X);
        SmartDashboard.putNumber("LimelightY", Limelight1.Y);
        SmartDashboard.putNumber("LimelightArea", Limelight1.A); 
    }



    // PRIVATE

    static void runLimelight(){
        Limelight1.update();
    }
}