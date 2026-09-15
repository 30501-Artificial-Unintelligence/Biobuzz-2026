package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "shooterTest")
public class shooterTest extends OpMode {
    DcMotor shooter;
    @Override
    public void init() {
       shooter = hardwareMap.get(DcMotor.class, "Shooter");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running");
        telemetry.update();
        shooter.setPower(0.5);

    }

}
