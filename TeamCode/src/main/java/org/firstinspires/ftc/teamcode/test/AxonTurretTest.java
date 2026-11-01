package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Axon MINI MK2 Turret Test", group = "Test")
public final class AxonTurretTest extends OpMode {
    private AxonTurret turret;
    private boolean homed, lastA;

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
        // X stops and requires another manual zero before enabling movement.
        if (gamepad1.x || turret.hasFault()) homed = false;
        if (gamepad1.a && !lastA && !homed && !gamepad1.x) {
            turret.zeroHere();
            homed = true;
        }
        lastA = gamepad1.a;
        if (homed) {
            if (gamepad1.dpad_up) turret.setTargetDegrees(0);
            if (gamepad1.dpad_right) turret.setTargetDegrees(90);
            if (gamepad1.dpad_left) turret.setTargetDegrees(-90);
            if (gamepad1.dpad_down) turret.setTargetDegrees(180);
        }
        // Hold right bumper as a deadman enable while testing.
        turret.update(homed && gamepad1.right_bumper && !gamepad1.x);
        telemetry.addData("Zero confirmed", homed);
        telemetry.addData("Turret degrees", turret.angleDegrees());
        telemetry.addData("Target degrees", turret.targetDegrees());
        telemetry.addData("Feedback volts", turret.voltage());
        telemetry.addData("Tracking fault: re-align and tap A", turret.hasFault());
        telemetry.addLine("A: confirm zero | hold RB: enable | X: stop/unhome");
        telemetry.addLine("D-pad: up 0, right +90, left -90, down +180");
    }

    @Override public void stop() {
        if (turret != null) turret.stop();
    }
}
