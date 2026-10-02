package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(5.9999607116218625);
        c.yPodOffset.set(0.517903012553538);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.16750210794500414);
                Controller secondaryTranslationalForward = Controller.proportional(0.061887553146923874);
                Controller primaryTranslationalLateral = Controller.proportional(0.2584848887254482);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09550325954062122);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013707941322453662));
                c.brake.set(Controller.proportionalFeedforward(0.011651750124085612));

                c.headingFeedback.set(Controller.proportional(3.2816556564948494));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0316720035914088, 0.014710801839656786));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05204641646041689, 0.061031498867760464));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0022354456136175053, 0.0016764397268831182));

                c.maxAchievableForwardVelocity.set(73.35401065075034);
                c.maxAchievableStrafeVelocity.set(61.960643772578464);
                c.naturalForwardDeceleration.set(39.368521661042195);
                c.naturalStrafeDeceleration.set(57.74451416437886);
            }
    );
}
