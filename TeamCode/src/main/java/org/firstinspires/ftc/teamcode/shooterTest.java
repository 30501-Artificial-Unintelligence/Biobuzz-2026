package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "shooterTest")
public class shooterTest extends OpMode {
    private final double Ticks = 28;
    private boolean gamepadRBumper = false;
    private boolean gamepadLBumper = false;

    private double TickTarget;
    double newTarget;
    boolean buttonB = gamepad1.b;
    // note to self: add into method later
    public double velocity = rpmConvert(20);



    double rpmConvert(double RPS) {
        TickTarget = RPS * Ticks;
        return TickTarget;
    }

    DcMotorEx shooter;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running");
        telemetry.update();
        Move(3);
    }
    public void Move(int Revolutions) {
        boolean currentRB = gamepad1.right_bumper;
        boolean currentLB = gamepad1.left_bumper;
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (gamepad1.right_bumper && !gamepadRBumper) {
            velocity -= rpmConvert(2);
        }
        if (gamepad1.left_bumper && !gamepadLBumper) {
            velocity += rpmConvert(2);
        }
        telemetry.addData("work", "ITS WORKING!!!");
        telemetry.update();
        shooter.setVelocity(velocity);
        gamepadRBumper = currentRB;
        gamepadLBumper = currentLB;
    }
}
        /*
        public void encoder(int turnage) {
          newTarget = Ticks/turnage;
          shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          shooter.setTargetPosition((int)newTarget);
          shooter.setPower(0.3);
          shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
         */