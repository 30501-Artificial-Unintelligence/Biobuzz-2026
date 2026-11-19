package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LightSubsystem {

    private final Servo rgbLight;

    // Keep these positions matched to your goBILDA LED configuration.
    private static final double OFF = 0.0;
    private static final double RED = 0.28;
    private static final double GREEN = 0.5;
    private static final double BLUE = 0.611;
    private static final double PURPLE = 0.7;

    public LightSubsystem(HardwareMap hardwareMap) {
        rgbLight = hardwareMap.get(Servo.class, "shooterRGB");
        turnOff();
    }

    // Reads the actual shooter state and speed through ShooterSubsystemFF.
    public void update(ShooterSubsystemFF shooter) {
        if (!shooter.isOn()) {
            turnOff();
        } else if (shooter.isReady()) {
            setGreen();
        } else {
            setRed();
        }
    }

    public void setRed() {
        rgbLight.setPosition(RED);
    }

    public void setGreen() {
        rgbLight.setPosition(GREEN);
    }

    public void setBlue() {
        rgbLight.setPosition(BLUE);
    }

    public void setPurple() {
        rgbLight.setPosition(PURPLE);
    }

    public void turnOff() {
        rgbLight.setPosition(OFF);
    }
}
