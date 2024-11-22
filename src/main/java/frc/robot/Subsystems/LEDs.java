package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
    Spark blinkin = new Spark(0);
    private double colour = Constants.ColourConstants.PINK;
    
    public LEDs() {

    }

    @Override
    public void periodic() {
        // blinkin.set(colour);
    }

    public Command setColour(double constant) {
        return Commands.runOnce(
            () -> {
                colour = constant;
            }
        );
    }
}
