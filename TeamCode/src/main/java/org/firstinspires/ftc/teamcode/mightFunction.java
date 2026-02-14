package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;

/**
 * FINAL COMPETITION CODE (Hardware PID Outtake)
 * Field-Centric Mecanum TeleOp with Control Hub mounted on 45-degree ramp.
 *
 * OUTTAKE MOTOR SPECS (goBILDA 6000 RPM):
 *   - Max free speed: 6000 RPM
 *   - Encoder PPR at output shaft: 28
 *   - Max ticks/sec: 28 * (6000/60) = 2800
 *
 * Outtake velocity control is handled by the REV hub's internal PIDF
 * controller at ~1000Hz via DcMotorEx.setVelocity(), which is faster
 * and more reliable than any software PID running in the ~50Hz Java loop.
 *
 * CONFIGURATION:
 *   - Hub Logo: DOWN
 *   - Hub USB:  LEFT
 *
 * SETUP:
 *   1. Place robot FLAT on floor.
 *   2. Press INIT, wait for "READY" (~3 seconds). Do NOT touch the robot.
 *   3. Press START.
 *
 * CONTROLS (gamepad1):
 *   Left Stick         — Translate (field-centric or robot-centric)
 *   Right Stick X      — Rotate
 *   Triggers (L/R)     — Precision Mode (analog, halves speed at full press)
 *   Options            — Reset field-centric heading (square up against wall first!)
 *   BACK               — EMERGENCY: Toggle Field-Centric / Robot-Centric
 *   Left Bumper        — Decrease outtake speed preset
 *   Right Bumper       — Increase outtake speed preset
 *   X Button           — Toggle primary intake on/off
 *   B Button           — Toggle secondary intake on/off
 */
@SuppressWarnings("unused")
@TeleOp(name = "Field Centric FINAL", group = "Competition")
public class mightFunction extends LinearOpMode {

    // =====================================================================
    // CONSTANTS
    // =====================================================================
    private static final double STRAFE_CORRECTION   = 1.1;
    private static final double JOYSTICK_DEADZONE   = 0.05;
    private static final long   IMU_SETTLE_MS       = 3000;
    private static final double PRECISION_MIN_SCALE = 0.5;

    // --- OUTTAKE MOTOR SPECS (goBILDA 6000 RPM, 28 PPR) ---
    private static final double OUTTAKE_TICKS_PER_REV = 28.0;

    /** Selectable outtake RPM presets. Index 0 = OFF.
     *  These are converted to ticks/sec and sent to the hub's internal
     *  PIDF controller via setVelocity(). */
    private static final double[] OUTTAKE_RPM_PRESETS = {
            0, 1300, 1400, 1500, 2000, 2200, 2300, 2400, 2500, 2600, 2700, 2800,
            2900, 3000, 3100, 3200, 3300
    };

    // =====================================================================
    // STATE VARIABLES
    // =====================================================================
    private Quaternion qMount        = null;
    private double     yawZeroOffset = 0.0;
    private int        outtakeSpeedIndex    = 0;
    private boolean    primaryIntakeActive   = false;
    private boolean    secondaryIntakeActive = false;

    // Drive Mode State
    private boolean    useFieldCentric = true;

    // Button Toggle Flags
    private boolean prevLeftBumper  = false;
    private boolean prevRightBumper = false;
    private boolean prevX           = false;
    private boolean prevB           = false;
    private boolean prevOptions     = false;
    private boolean prevBack        = false;

    @Override
    public void runOpMode() {

        // -----------------------------------------------------------------
        // 1. PERFORMANCE: MANUAL BULK CACHING
        // -----------------------------------------------------------------
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // -----------------------------------------------------------------
        // 2. HARDWARE MAPPING
        // -----------------------------------------------------------------
        DcMotor frontLeft  = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeft   = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRight  = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor primaryIntake   = hardwareMap.dcMotor.get("intake");
        DcMotor secondaryIntake = hardwareMap.dcMotor.get("SecondIntake");

        // Outtake motors as DcMotorEx for setVelocity() access
        DcMotorEx leftOuttake  = hardwareMap.get(DcMotorEx.class, "leftOuttake");
        DcMotorEx rightOuttake = hardwareMap.get(DcMotorEx.class, "rightOuttake");

        // -----------------------------------------------------------------
        // 3. IMU INITIALIZATION
        // -----------------------------------------------------------------
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));

        // -----------------------------------------------------------------
        // 4. MOTOR CONFIGURATION
        // -----------------------------------------------------------------

        // Drivetrain
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Intakes
        primaryIntake.setDirection(DcMotor.Direction.REVERSE);

        // Outtake: Reset encoders, then enable the hub's internal PIDF
        // velocity controller via RUN_USING_ENCODER. This offloads speed
        // control to the hub's ~1000Hz loop instead of our ~50Hz Java loop.
        rightOuttake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // FLOAT (coast) for flywheels — BRAKE would fight the flywheel's
        // inertia and send back-EMF voltage spikes into the hub.
        leftOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // -----------------------------------------------------------------
        // 5. CALIBRATION
        // -----------------------------------------------------------------
        telemetry.addData("STATUS", "CALIBRATING - DO NOT TOUCH");
        telemetry.addData("INFO",   "Place robot FLAT on floor.");
        telemetry.update();
        sleep(IMU_SETTLE_MS);

        qMount = imu.getRobotOrientationAsQuaternion();

        telemetry.addData("STATUS", "READY");
        telemetry.addData("Mount", formatQuaternion(qMount));
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        ElapsedTime loopTimer = new ElapsedTime();

        // =================================================================
        // MAIN LOOP
        // =================================================================
        while (opModeIsActive()) {

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            // -------------------------------------------------------------
            // DRIVE MODE TOGGLE (BACK BUTTON)
            // -------------------------------------------------------------
            if (gamepad1.back && !prevBack) {
                useFieldCentric = !useFieldCentric;
            }
            prevBack = gamepad1.back;

            // -------------------------------------------------------------
            // SENSOR READING & HEADING CALC
            // -------------------------------------------------------------
            Quaternion qHub   = imu.getRobotOrientationAsQuaternion();
            Quaternion qRobot = multiply(qHub, inverse(qMount));

            double fwdX = 1.0 - 2.0 * (qRobot.y * qRobot.y + qRobot.z * qRobot.z);
            double fwdY = 2.0 * (qRobot.x * qRobot.y + qRobot.w * qRobot.z);
            double botHeading = Math.atan2(fwdY, fwdX);

            if (gamepad1.options && !prevOptions) {
                yawZeroOffset = botHeading;
            }
            prevOptions = gamepad1.options;

            double fieldHeading = Math.atan2(
                    Math.sin(botHeading - yawZeroOffset),
                    Math.cos(botHeading - yawZeroOffset)
            );

            // -------------------------------------------------------------
            // DRIVETRAIN MATH
            // -------------------------------------------------------------
            double rawDrive  = applyDeadzone(-gamepad1.left_stick_y,  JOYSTICK_DEADZONE);
            double rawStrafe = applyDeadzone( gamepad1.left_stick_x,  JOYSTICK_DEADZONE);
            double rawTurn   = applyDeadzone(-gamepad1.right_stick_x, JOYSTICK_DEADZONE);

            double drive  = Math.copySign(rawDrive  * rawDrive,  rawDrive);
            double strafe = Math.copySign(rawStrafe * rawStrafe, rawStrafe) * STRAFE_CORRECTION;
            double turn   = Math.copySign(rawTurn   * rawTurn,   rawTurn);

            double triggerValue = Math.max(gamepad1.left_trigger, gamepad1.right_trigger);
            double speedScale   = 1.0 - triggerValue * (1.0 - PRECISION_MIN_SCALE);

            drive  *= speedScale;
            strafe *= speedScale;
            turn   *= speedScale;

            double rotatedX = strafe;
            double rotatedY = drive;

            if (useFieldCentric) {
                rotatedX = strafe * Math.cos(-fieldHeading) - drive * Math.sin(-fieldHeading);
                rotatedY = strafe * Math.sin(-fieldHeading) + drive * Math.cos(-fieldHeading);
            }

            double denom = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(turn), 1.0);
            frontLeft.setPower( (rotatedY + rotatedX + turn) / denom);
            backLeft.setPower(  (rotatedY - rotatedX + turn) / denom);
            frontRight.setPower((rotatedY - rotatedX - turn) / denom);
            backRight.setPower( (rotatedY + rotatedX - turn) / denom);

            // -------------------------------------------------------------
            // OUTTAKE VELOCITY CONTROL (Hardware PIDF)
            // -------------------------------------------------------------
            if (gamepad1.left_bumper && !prevLeftBumper) {
                outtakeSpeedIndex = Math.max(0, outtakeSpeedIndex - 1);
            }
            prevLeftBumper = gamepad1.left_bumper;

            if (gamepad1.right_bumper && !prevRightBumper) {
                outtakeSpeedIndex = Math.min(OUTTAKE_RPM_PRESETS.length - 1, outtakeSpeedIndex + 1);
            }
            prevRightBumper = gamepad1.right_bumper;

            double targetRPM = OUTTAKE_RPM_PRESETS[outtakeSpeedIndex];

            // Convert RPM → ticks/sec and send to the hub's internal controller.
            // Each motor is controlled independently — if one encoder fails,
            // the other motor is unaffected.
            double targetTPS = targetRPM / 60.0 * OUTTAKE_TICKS_PER_REV;

            if (targetRPM == 0) {
                // Explicitly set power to 0 instead of setVelocity(0) so the
                // motors coast to a stop (FLOAT behavior) rather than the
                // PIDF actively braking to hold 0 velocity.
                leftOuttake.setPower(0);
                rightOuttake.setPower(0);
            } else {
                leftOuttake.setVelocity(targetTPS);
                rightOuttake.setVelocity(targetTPS);
            }

            // Read actual velocities for telemetry
            double actualLeftTPS  = leftOuttake.getVelocity();
            double actualRightTPS = rightOuttake.getVelocity();
            double actualLeftRPM  = actualLeftTPS  / OUTTAKE_TICKS_PER_REV * 60.0;
            double actualRightRPM = actualRightTPS / OUTTAKE_TICKS_PER_REV * 60.0;

            // -------------------------------------------------------------
            // INTAKE TOGGLES
            // -------------------------------------------------------------
            if (gamepad1.x && !prevX) {
                primaryIntakeActive = !primaryIntakeActive;
                primaryIntake.setPower(primaryIntakeActive ? 1.0 : 0.0);
            }
            prevX = gamepad1.x;

            if (gamepad1.b && !prevB) {
                secondaryIntakeActive = !secondaryIntakeActive;
                secondaryIntake.setPower(secondaryIntakeActive ? 1.0 : 0.0);
            }
            prevB = gamepad1.b;

            // -------------------------------------------------------------
            // TELEMETRY
            // -------------------------------------------------------------
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            telemetry.addData("DRIVE MODE", useFieldCentric
                    ? "FIELD-CENTRIC"
                    : ">>> ROBOT-CENTRIC <<<");
            if (useFieldCentric) {
                telemetry.addData("Heading", "%.1f deg", Math.toDegrees(fieldHeading));
            }
            telemetry.addData("Speed",     "%.0f%%", speedScale * 100.0);
            telemetry.addData("Loop",      "%.1f ms", loopMs);
            telemetry.addLine();
            telemetry.addData("Outtake Target", "%.0f RPM [%d/%d]",
                    targetRPM, outtakeSpeedIndex, OUTTAKE_RPM_PRESETS.length - 1);
            telemetry.addData("Outtake Left",   "%.0f RPM", actualLeftRPM);
            telemetry.addData("Outtake Right",  "%.0f RPM", actualRightRPM);
            telemetry.addLine();
            telemetry.addData("Intake 1",  primaryIntakeActive   ? "ON" : "OFF");
            telemetry.addData("Intake 2",  secondaryIntakeActive ? "ON" : "OFF");
            telemetry.update();
        }
    }

    // =====================================================================
    // MATH HELPERS
    // =====================================================================

    private Quaternion multiply(Quaternion a, Quaternion b) {
        return new Quaternion(
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                System.nanoTime()
        );
    }

    private Quaternion inverse(Quaternion q) {
        return new Quaternion(q.w, -q.x, -q.y, -q.z, q.acquisitionTime);
    }

    private String formatQuaternion(Quaternion q) {
        return String.format("(%.2f, %.2f, %.2f, %.2f)", q.w, q.x, q.y, q.z);
    }

    private double applyDeadzone(double inp4ut, double threshold) {
        if (Math.abs(input) < threshold) return 0.0;
        return Math.copySign(
                (Math.abs(input) - threshold) / (1.0 - threshold),
                input
        );
    }
}