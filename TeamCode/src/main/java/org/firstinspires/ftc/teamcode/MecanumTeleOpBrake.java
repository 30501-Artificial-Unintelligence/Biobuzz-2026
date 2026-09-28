package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Robot-centric mecanum teleop with BRAKE mode on every drive motor.
 *
 * Controls:
 *   Left stick  → forward/back + strafe
 *   Right stick X → rotate
 *
 * Hardware map names (change these to match your Robot Configuration):
 *   frontLeft, backLeft, frontRight, backRight
 */
@TeleOp(name = "Mecanum TeleOp (Brake)", group = "TeleOp")
public class MecanumTeleOpBrake extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== Hardware Map =====
        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // ===== Directions =====
        // Reverse the right side so positive power drives forward.
        // If the robot drives backward when you push the stick forward,
        // reverse the LEFT side instead.
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // ===== BRAKE mode on every drive motor =====
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run without encoders (standard for pure teleop)
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized – BRAKE mode active");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // ===== Gamepad Input =====
            double y  = -gamepad1.left_stick_y;          // Forward / back (Y is inverted)
            double x  =  gamepad1.left_stick_x * 1.1;    // Strafe (1.1 helps imperfect rollers)
            double rx =  gamepad1.right_stick_x;         // Rotate

            // ===== Normalized mecanum math =====
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            // ===== Apply power =====
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Optional live telemetry
            telemetry.addData("FL", "%.2f", frontLeftPower);
            telemetry.addData("BL", "%.2f", backLeftPower);
            telemetry.addData("FR", "%.2f", frontRightPower);
            telemetry.addData("BR", "%.2f", backRightPower);
            telemetry.update();
        }
    }
}