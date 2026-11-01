package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/** Axon MINI MK2 in CR mode. Call update() every loop, even while disabled. */
public final class AxonTurret {
    // goBILDA 3208-0004-0001 with its included 64T drive gear.
    // Assumes the servo directly drives that gear without an extra gear stage.
    public static final double RATIO = 2.75; // servo turns per turret turn

    // Provisional calibration: verify the feedback voltage span on your hardware.
    public static double FEEDBACK_MIN_VOLTS = 0.0;
    public static double FEEDBACK_MAX_VOLTS = 3.3;
    // Provisional direction: verify on the assembled turret and flip if reversed.
    public static double ENCODER_TO_TURRET_SIGN = -1;
    // Flip this if positive power makes measured turret angle DECREASE.
    public static double POWER_SIGN = -1;

    // MINI MK2 starter gains only; tune on the assembled mechanism.
    // Units are power per degree, second, etc.
    public double kP = 0.01, kI = 0.0, kD = 0.0;
    public double maxPower = 0.20;
    public double toleranceDegrees = 1.0;
    // Provisional cable-safe limits relative to the manually established zero.
    public double minDegrees = -180, maxDegrees = 180;

    private final CRServo servo;
    private final AnalogInput feedback;
    private double previousRaw, accumulatedServo, zeroOffset, target;
    private double integral, previousAngle;
    private long previousTime;
    private boolean fault;

    public AxonTurret(HardwareMap map) {
        servo = map.get(CRServo.class, "turretServo");
        feedback = map.get(AnalogInput.class, "turretEncoder");
        servo.setPower(0);
        previousRaw = rawDegrees();
        previousTime = System.nanoTime();
    }

    private double rawDegrees() {
        return (feedback.getVoltage() - FEEDBACK_MIN_VOLTS)
                / (FEEDBACK_MAX_VOLTS - FEEDBACK_MIN_VOLTS) * 360.0;
    }

    /** Only call with the turret physically aligned to your known zero and stopped. */
    public void zeroHere() {
        servo.setPower(0);
        previousRaw = rawDegrees();
        accumulatedServo = 0;
        zeroOffset = 0;
        target = 0;
        previousAngle = 0;
        integral = 0;
        previousTime = System.nanoTime();
        fault = false;
    }

    public double angleDegrees() {
        return ENCODER_TO_TURRET_SIGN * accumulatedServo / RATIO + zeroOffset;
    }

    /** Uses an unwrapped target: 0 to 360 commands a whole turn, if limits allow it. */
    public void setTargetDegrees(double degrees) {
        target = Range.clip(degrees, minDegrees, maxDegrees);
    }

    public double targetDegrees() { return target; }
    public double voltage() { return feedback.getVoltage(); }
    public boolean hasFault() { return fault; }

    public void update(boolean enabled) {
        long now = System.nanoTime();
        double dt = (now - previousTime) * 1e-9;
        previousTime = now;
        double raw = rawDegrees();
        double delta = raw - previousRaw;
        previousRaw = raw;
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;
        // A long gap may hide complete servo revolutions. Re-home after a fault.
        if (dt > 0.10 || !Double.isFinite(raw)) fault = true;
        accumulatedServo += delta;
        double angle = angleDegrees();
        double velocity = dt > 0.000001 ? (angle - previousAngle) / dt : 0;
        previousAngle = angle;

        if (!enabled || fault || dt <= 0.000001) {
            stop();
            return;
        }
        double error = target - angle;
        if (Math.abs(error) <= toleranceDegrees) {
            stop();
            return;
        }
        // Derivative on measurement avoids a kick when changing the target.
        double candidateIntegral = integral + error * dt;
        double output = kP * error + kI * candidateIntegral - kD * velocity;
        if (Math.abs(output) <= maxPower) integral = candidateIntegral;
        output = Range.clip(kP * error + kI * integral - kD * velocity,
                -maxPower, maxPower);
        if ((angle >= maxDegrees && output > 0)
                || (angle <= minDegrees && output < 0)) output = 0;
        servo.setPower(POWER_SIGN * output);
    }

    public void stop() {
        servo.setPower(0);
        integral = 0;
    }
}
