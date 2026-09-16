package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "shooterTest")
public class shooterTest extends OpMode {
    private final double Ticks = 28;
    double newTarget;
    boolean buttonB = gamepad1.b;
    DcMotor shooter;
    @Override
    public void init() {
       shooter = hardwareMap.get(DcMotor.class, "Shooter");
       shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running");
        telemetry.update();
        if (buttonB) {
            encoder(2);
        }

        }
        public void encoder(int turnage) {
          newTarget = Ticks/turnage;
          shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          shooter.setTargetPosition((int)newTarget);
          shooter.setPower(0.3);
          shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
