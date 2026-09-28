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
    // note to self: add into method later
    public int CurrentRPM = 2500;
    public double velocity = rpmConvert(CurrentRPM);




    double rpmConvert(double RPM) {
        TickTarget = RPM/60 * Ticks;
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
        Move();
    }
    public void Move() {
        if (gamepad1 == null) {
            return;
        } else {
            boolean currentRB = gamepad1.right_bumper;
            boolean currentLB = gamepad1.left_bumper;
            if (gamepad1.right_bumper && !gamepadRBumper) {
                velocity -= rpmConvert(50);
                CurrentRPM -= 50;

            }
            if (gamepad1.left_bumper && !gamepadLBumper) {
                velocity += rpmConvert(50);
                CurrentRPM +=50;

            }
            telemetry.addData("work", "ITS WORKING!!!");
            double actualRPM = shooter.getVelocity() * 60.0 / 28.0;
            telemetry.addData("Target RPM", CurrentRPM);
            telemetry.addData("Actual RPM", "%.0f", actualRPM);
            telemetry.update();
            shooter.setVelocity(velocity);
            gamepadRBumper = currentRB;
            gamepadLBumper = currentLB;
        }
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