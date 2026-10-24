package org.firstinspires.ftc.teamcode.testingTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Stopper Servo Test")
public class StopperServoTest extends OpMode {

    private Servo stopper;

    private double position = 0.5;

    // Servo position change per second
    private static final double SPEED = 0.3;

    // Save positions for stopper subsystem
    private double onPos = 0.5;
    private double offPos = 0.5;

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        stopper = hardwareMap.get(Servo.class, "stopper");

        stopper.setPosition(position);
        timer.reset();
    }

    @Override
    public void loop() {

        double dt = Math.min(timer.seconds(), 0.1);
        timer.reset();

        // Left stick up = increase position
        // Left stick down = decrease position
        double input = -gamepad1.left_stick_y;

        if (Math.abs(input) > 0.05) {
            position += input * SPEED * dt;
        }

        // Clamp position between 0 and 1
        position = Range.clip(position, 0.0, 1.0);

        // Command servo
        stopper.setPosition(position);

        // Save ON position
        if (gamepad1.a) {
            onPos = position;
        }

        // Save OFF position
        if (gamepad1.b) {
            offPos = position;
        }

        // Telemetry
        telemetry.addData("Current Position", "%.4f", position);
        telemetry.addData("Saved ON Position", "%.4f", onPos);
        telemetry.addData("Saved OFF Position", "%.4f", offPos);

        telemetry.addLine();
        telemetry.addLine("Left Stick: Adjust servo");
        telemetry.addLine("A: Save ON position");
        telemetry.addLine("B: Save OFF position");

        telemetry.update();
    }
}