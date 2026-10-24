
package org.firstinspires.ftc.teamcode.testingTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test")
public class ServoTest extends OpMode {

    Servo servo;
    double position = 0.5;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "stopper");
    }

    @Override
    public void loop() {
        position -= gamepad1.left_stick_y * 0.005;
        position = Math.max(0, Math.min(1, position));

        servo.setPosition(position);

        telemetry.addData("Servo Position", position);
        telemetry.update();
    }
}
