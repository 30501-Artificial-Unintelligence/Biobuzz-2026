package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(new PinpointLocalizer(h, localizerConfig), new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig));

    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backRight");
        c.backRightName.set("backLeft");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.12032280756732612);
        c.yPodOffset.set(0.5383128068578525);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.23263128884117465);
                Controller secondaryTranslationalForward = Controller.proportional(0.08595104520429407);
                Controller primaryTranslationalLateral = Controller.proportional(0.3347617810695426);
                Controller secondaryTranslationalLateral = Controller.proportional(0.12368553310566338);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011590410666432342));
                c.brake.set(Controller.proportionalFeedforward(0.00985184906646749));

                c.headingFeedback.set(Controller.proportional(4.170883107428792));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06884847830565878, 0.006254194576378065));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11585051151133421, 0.08687689814536033));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001094001519131359, 0.0014977123068314851));

                c.maxAchievableForwardVelocity.set(85.97412337729699);
                c.maxAchievableStrafeVelocity.set(69.30029268147734);
                c.naturalForwardDeceleration.set(37.76495317314023);
                c.naturalStrafeDeceleration.set(69.32681714763285);
            }
    );
}
