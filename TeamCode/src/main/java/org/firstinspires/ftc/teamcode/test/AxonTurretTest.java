package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Axon MINI MK2 Turret Test", group = "Test")
public final class AxonTurretTest extends OpMode {
    private AxonTurret turret;
    private boolean homed, lastA;
    private boolean lastUp, lastDown, lastLeft, lastRight;
    private int selectedGain;
    private static final String[] GAIN_NAMES = {"P", "I", "D"};
    private static final double[] GAIN_STEPS = {0.001, 0.0001, 0.0001};

    @Override public void init() {
        turret = new AxonTurret(hardwareMap);
        telemetry.addLine("Align turret to forward zero. After START, tap A to confirm.");
    }

    @Override public void init_loop() {
        turret.update(false);
        telemetry.addData("Feedback volts", turret.voltage());
    }

    @Override public void start() {
        // Tracking reference only. Driver must still confirm the physical zero with A.
        turret.zeroHere();
    }

    @Override public void loop() {
        boolean tuning = gamepad1.left_bumper;
        // Edge detection: each press changes a value once, regardless of loop rate.
        if (tuning) {
            if (gamepad1.dpad_left && !lastLeft)
                selectedGain = (selectedGain + 2) % 3;
            if (gamepad1.dpad_right && !lastRight)
                selectedGain = (selectedGain + 1) % 3;
            double change = 0;
            if (gamepad1.dpad_up && !lastUp) change += GAIN_STEPS[selectedGain];
            if (gamepad1.dpad_down && !lastDown) change -= GAIN_STEPS[selectedGain];
            if (gamepad1.y) change *= 10; // Coarse adjustment.
            if (change != 0) {
                if (selectedGain == 0) turret.kP = Math.max(0, turret.kP + change);
                if (selectedGain == 1) turret.kI = Math.max(0, turret.kI + change);
                if (selectedGain == 2) turret.kD = Math.max(0, turret.kD + change);
            }
        }
        // X stops and requires another manual zero before enabling movement.
        if (gamepad1.x || turret.hasFault()) homed = false;
        if (gamepad1.a && !lastA && !homed && !gamepad1.x) {
            turret.zeroHere();
            homed = true;
        }
        lastA = gamepad1.a;
        if (homed && !tuning) {
            if (gamepad1.dpad_up) turret.setTargetDegrees(0);
            if (gamepad1.dpad_right) turret.setTargetDegrees(90);
            if (gamepad1.dpad_left) turret.setTargetDegrees(-90);
            if (gamepad1.dpad_down) turret.setTargetDegrees(180);
        }
        // Hold right bumper as a deadman enable while testing.
        // Tuning pauses output and clears the integral, even when RB is held.
        turret.update(homed && gamepad1.right_bumper && !gamepad1.x && !tuning);
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        telemetry.addData("PID", "P %.5f | I %.5f | D %.5f", turret.kP, turret.kI, turret.kD);
        telemetry.addData("Selected gain", GAIN_NAMES[selectedGain]);
        telemetry.addData("Adjustment step", "%.5f (hold Y for 10x)", GAIN_STEPS[selectedGain]);
        telemetry.addData("Tuning / motion paused", tuning);
        telemetry.addData("Zero confirmed", homed);
        telemetry.addData("Turret degrees", turret.angleDegrees());
        telemetry.addData("Target degrees", turret.targetDegrees());
        telemetry.addData("Feedback volts", turret.voltage());
        telemetry.addData("Tracking fault: re-align and tap A", turret.hasFault());
        telemetry.addLine("A: confirm zero | hold RB: enable | X: stop/unhome");
        telemetry.addLine("D-pad: up 0, right +90, left -90, down +180");
        telemetry.addLine("Hold LB: left/right select P/I/D; up/down adjust; Y = 10x");
        telemetry.addLine("Gains reset on OpMode initialization. Record your tuned values.");
    }

    @Override public void stop() {
        if (turret != null) turret.stop();
    }
}
