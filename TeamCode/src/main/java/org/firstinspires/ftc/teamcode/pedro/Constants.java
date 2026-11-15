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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



/** Pedro 3.0.1 configuration. Replace localizer/Foresight entries with AutoTune output. */
public final class Constants {
    private Constants() {}

    public static boolean LOCALIZER_CALIBRATED = false;
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL");
        c.frontRightName.set("FR");
        c.backLeftName.set("BL");
        c.backRightName.set("BR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    // Pinpoint selected from the AU26 design. Zero offsets are provisional and are not accepted for
    // field aiming/pathing.
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-1.563221203060601);
        c.yPodOffset.set(-4.599506588432733);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

//    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
//        c.name.set("pinpoint");
//        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        c.xPodOffset.set(-2.0699890016570803);
//        c.yPodOffset.set(-4.717289481576033);
//        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        c.globalDistanceUnit.set(DistanceUnit.INCH);
//        c.offsetUnits.set(DistanceUnit.INCH);
//    });

//    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
//        c.name.set("pinpoint");
//        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        c.xPodOffset.set(-1.4477150083526853);
//        c.yPodOffset.set(-4.460574472983052);
//        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        c.globalDistanceUnit.set(DistanceUnit.INCH);
//        c.offsetUnits.set(DistanceUnit.INCH);
//    });

//    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
//        c.name.set("pinpoint");
//        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        c.xPodOffset.set(-1.3426357179176152);
//        c.yPodOffset.set(-4.365029823123002);
//        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        c.globalDistanceUnit.set(DistanceUnit.INCH);
//        c.offsetUnits.set(DistanceUnit.INCH);
//    });

//    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
//        c.name.set("pinpoint");
//        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        c.xPodOffset.set(-1.0949897015188623);
//        c.yPodOffset.set(-4.878626845953033);
//        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        c.globalDistanceUnit.set(DistanceUnit.INCH);
//        c.offsetUnits.set(DistanceUnit.INCH);
//    });



    // Paste the Foresight AutoTune-generated ForesightConfig here. Manual drive works without an
    // algorithm.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.1795483906942892);
                Controller secondaryTranslationalForward = Controller.proportional(0.06633833274018147);
                Controller primaryTranslationalLateral = Controller.proportional(0.2588701553449293);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09564560526200581);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011105801965434344));
                c.brake.set(Controller.proportionalFeedforward(0.009439931670619192));

                c.headingFeedback.set(Controller.proportional(2.9048521105454297));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.052165612535102775, 0.0051378007958380115));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07659918290139214, 0.06338305517812595));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001198334297398328, 0.0014398452576794849));

                c.maxAchievableForwardVelocity.set(78.85775610193488);
                c.maxAchievableStrafeVelocity.set(65.6576480348022);
                c.naturalForwardDeceleration.set(49.73508910013442);
                c.naturalStrafeDeceleration.set(67.49775945754106);
            }
    );

    public static boolean pathingCalibrated() {
        return LOCALIZER_CALIBRATED && foresightConfig != null;
    }

    public static PinpointLocalizer createLocalizer(HardwareMap map) {
        return new PinpointLocalizer(map, localizerConfig);
    }

    public static Mecanum createDrivetrain(HardwareMap map) {
        String[] names = {
                drivetrainConfig.frontLeftName.get(),
                drivetrainConfig.backLeftName.get(),
                drivetrainConfig.frontRightName.get(),
                drivetrainConfig.backRightName.get()
        };
        for (String name : names) {
            DcMotorEx motor = map.get(DcMotorEx.class, name);
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        return new Mecanum(map, drivetrainConfig);
    }

    public static Follower create(HardwareMap map) {
        Follower follower =
                new Follower(
                        createLocalizer(map),
                        createDrivetrain(map),
                        pathingCalibrated() ? new Foresight(foresightConfig) : null);
        follower.holdEnd.set(false);
        return follower;
    }
}