package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase{

    AnalogOutput red = new AnalogOutput(0);
    AnalogOutput green = new AnalogOutput(1);
    //AnalogOutput blue = new AnalogOutput(1);

    double redVoltage = 0;
    double greenVoltage = 0;
    double blueVoltage = 0;

    Timer ledTimer;
    
    private static LEDs LEDS;

    @Override
    public void periodic() {
        red.setVoltage(redVoltage);
        green.setVoltage(greenVoltage);
        //blue.setVoltage(blueVoltage);
    }

    public LEDs() {
        if(isRed()) {
            redVoltage = 2;
            greenVoltage = 0;
            blueVoltage = 0;
        } else {
            redVoltage = 0;
            greenVoltage = 0;
            blueVoltage = 2;
        }

        ledTimer = new Timer();
        ledTimer.reset();
        ledTimer.start();
    }

    public static LEDs getInstance() {
        if(LEDS == null) {
            LEDS = new LEDs();
        }
        return LEDS;
    }

    public Command staticNormal() {
        return Commands.runOnce(() -> {
        if(isRed()) {
            redVoltage = 2;
            greenVoltage = 0;
            blueVoltage = 0;
        } else {
            redVoltage = 0;
            greenVoltage = 0;
            blueVoltage = 2;
        }
        });
    }
  
    public Command signalEndgame() {
        return Commands.runEnd(
            () -> {
                if(ledTimer.get() < 0.5) { //green flash
                    redVoltage = 0;
                    greenVoltage = 2;
                    blueVoltage = 0;
                } else if (ledTimer.get() < 1) { //TODO adjust timings
                    redVoltage = 0;
                    greenVoltage = 0;
                    blueVoltage = 0;
                } else {
                    ledTimer.reset();
                }
            },
            () -> staticNormal()
        );
    }

    public Command signalIntake() {
        return Commands.runEnd(
            () -> {
                if(ledTimer.get() < 0.5) { //blue flash
                    redVoltage = 0;
                    greenVoltage = 0;
                    blueVoltage = 2;
                } else if (ledTimer.get() < 1) { //TODO adjust timings
                    redVoltage = 0;
                    greenVoltage = 0;
                    blueVoltage = 0;
                } else {
                    ledTimer.reset();
                }
        },
        () -> staticNormal());
    }

    public Command signalAmplify() {
        return Commands.runEnd(
            () -> {
                if(ledTimer.get() < 0.5) { //purple flash
                    redVoltage = 2;
                    greenVoltage = 0;
                    blueVoltage = 2;
                } else if (ledTimer.get() < 1) { //TODO adjust timings
                    redVoltage = 0;
                    greenVoltage = 0;
                    blueVoltage = 0;
                } else {
                    ledTimer.reset();
                }
        },
        () -> staticNormal());
    }

    public Command resetTimer() {
        return Commands.runOnce(() -> {
            ledTimer.reset();
        });
    }

    public boolean isRed() {
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    enum LEDState{
        ENDGAME, LOADED, AMPLIFY;
    }


}
