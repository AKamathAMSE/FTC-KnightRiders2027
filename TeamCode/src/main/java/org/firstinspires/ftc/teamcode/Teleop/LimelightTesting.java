/*
Current Controls(Subject to Change)
 * Left Stick (Up / Down)    : Drive forward and backward (field-oriented)
 * Left Stick (Left / Right) : Strafe left and right (field-oriented)
 * Right Stick (Left / Right): Rotate clockwise and counter-clockwise
 *
 * D-Pad Left                : Set Blue Alliance (BLUE_GOAL_TAG_ID)
 * D-Pad Right               : Set Red Alliance (RED_GOAL_TAG_ID + 180 deg flip)
 * Back / Select Button      : Reset forward field heading to 0 degrees
 *
 * Right Trigger             : Run intake forward (1.0) and open Gate (0.5)
 * B Button                  : Spin shooter at auto-RPM (or 3500 fallback)
 *                             and reverse intake (-1.0)
 * Left Trigger              : Run shooter in reverse (-2500 ticks/sec)
 *
 * Y Button                  : Move arm to 90 degrees
 * X Button                  : Move arm to 45 degrees
 * A Button                  : Move arm to 0 degrees (home)

 ADVIK PLEASE DO NOT TOUCH THE CODE
 ADVATE PLEASE MAKE SURE ADVIK DOESNT TOUCH THE CODE
 AIDAN PLEASE MAKE SURE ADVIK DOESNT TOUCH THE CODE
 */
package org.firstinspires.ftc.teamcode.Teleop;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Config
@TeleOp(name = "LimelightTesting")
public class LimelightTesting extends LinearOpMode {
    private DcMotorEx LB, LF, RB, RF;
    private Servo Gate;
    private DcMotorEx Shooterl, Shooterr;
    private DcMotorEx Intakel, Intaker;
    private CRServo Arm;

    private Limelight3A limelight;
    private IMU imu;
    private double headingOffset = 0.0;
    private double currentFieldHeadingDeg = 0.0;
    private boolean isRedAlliance = false;

    public static int APRILTAG_PIPELINE = 0;
    public static int BLUE_GOAL_TAG_ID = 7;
    public static int RED_GOAL_TAG_ID = 4;
    public static double LIMELIGHT_MOUNT_ANGLE_DEG = 25.0;
    public static double LIMELIGHT_LENS_HEIGHT_IN = 10.0;
    public static double TARGET_TAG_HEIGHT_IN = 36.0;
    public static double MAX_YAW_JUMP_DEG = 45.0;

    public static double SHOOTER_TICKS_PER_REV = 28.0;
    public static double MIN_SHOOTER_RPM = 2500.0;
    public static double MAX_SHOOTER_RPM = 5200.0;
    public static double FALLBACK_SHOOTER_RPM = 3500.0;
    public static double BASE_SHOOTER_RPM = 2200.0;
    public static double RPM_PER_INCH = 15.0;
    public static double RPM_PER_INCH_SQUARED = 0.03;

    double lbPower, lfPower, rbPower, rfPower;
    double ilPower, irPower;

    public static double ARM_POS_1 = 0.482;
    public static double ARM_POS_2 = 0.4928;
    public static double ARM_POS_3 = 0.5036;
    public static double armTarget = 0.0;

    private ElapsedTime armTimer = new ElapsedTime();
    private boolean isArmMoving = false;
    private double armTargetTime = 0.0;
    public static double DEGREES_PER_SECOND = 90.0;
    public static double ARM_POWER = 0.6;
    private double currentEstimatedAngle = 0.0;
    private double targetAngle = 0.0;

    public static double GATE_HOME_POSITION = 0.0;
    public static double GATE_CLOCKWISE_POSITION = 0.5;

    public static double shooterP = 0.001;
    public static double shooterI = 0.0;
    public static double shooterD = 0.0;
    public static double shooterF = 0.0002;

    private double targetShooterVelocity = 0.0;
    private double calculatedDistanceInches = 0.0;
    private boolean hasValidGoalTag = false;
    private int detectedTagId = -1;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        LB = hardwareMap.get(DcMotorEx.class, "LB");
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        RB = hardwareMap.get(DcMotorEx.class, "RB");
        RF = hardwareMap.get(DcMotorEx.class, "RF");

        Arm = hardwareMap.get(CRServo.class, "Arm");
        Gate = hardwareMap.get(Servo.class, "Gate");

        Intakel = hardwareMap.get(DcMotorEx.class, "Intakel");
        Intaker = hardwareMap.get(DcMotorEx.class, "Intaker");
        Shooterr = hardwareMap.get(DcMotorEx.class, "Shooterr");
        Shooterl = hardwareMap.get(DcMotorEx.class, "Shooterl");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(imuParameters);
        imu.resetYaw();

        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.REVERSE);

        Intakel.setDirection(DcMotor.Direction.FORWARD);
        Intaker.setDirection(DcMotor.Direction.REVERSE);
        Shooterl.setDirection(DcMotor.Direction.FORWARD);
        Shooterr.setDirection(DcMotor.Direction.REVERSE);

        Arm.setDirection(CRServo.Direction.FORWARD);
        Gate.setDirection(Servo.Direction.FORWARD);

        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intakel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intaker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooterl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooterr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intakel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intaker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooterl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooterr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Gate.setPosition(GATE_HOME_POSITION);

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_right) {
                isRedAlliance = true;
            } else if (gamepad1.dpad_left) {
                isRedAlliance = false;
            }
            telemetry.addData("Status", "Initialized - Select Alliance with D-Pad Left/Right");
            telemetry.addData("Selected Alliance", isRedAlliance ? "RED (Target ID " + RED_GOAL_TAG_ID + ")" : "BLUE (Target ID " + BLUE_GOAL_TAG_ID + ")");
            telemetry.update();
        }

        runtime.reset();
        BasicPIDController shooterPID = new BasicPIDController(shooterP, shooterI, shooterD);

        while (opModeIsActive()) {
            shooterPID.setPID(shooterP, shooterI, shooterD);

            if (gamepad1.dpad_right) {
                isRedAlliance = true;
            } else if (gamepad1.dpad_left) {
                isRedAlliance = false;
            }
            if (gamepad1.back) {
                imu.resetYaw();
                headingOffset = 0.0;
            }

            updateVisionAndHeading();

            intakeMethod();
            shooterMethod(shooterPID);
            armControlMethod();
            gateControlMethod();

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double headingRad = Math.toRadians(currentFieldHeadingDeg);
            double rotForward = forward * Math.cos(-headingRad) - strafe * Math.sin(-headingRad);
            double rotStrafe = forward * Math.sin(-headingRad) + strafe * Math.cos(-headingRad);

            lbPower = rotForward - rotStrafe + turn;
            lfPower = rotForward + rotStrafe + turn;
            rbPower = rotForward + rotStrafe - turn;
            rfPower = rotForward - rotStrafe - turn;

            double max = Math.max(
                    Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                    Math.max(Math.abs(lbPower), Math.abs(rbPower))
            );
            if (max > 1.0) {
                lbPower /= max;
                lfPower /= max;
                rbPower /= max;
                rfPower /= max;
            }

            LB.setPower(lbPower);
            LF.setPower(lfPower);
            RB.setPower(rbPower);
            RF.setPower(rfPower);

            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Field Heading (Deg)", String.format("%.1f", currentFieldHeadingDeg));
            telemetry.addData("Goal Tag Visible", hasValidGoalTag + " (ID: " + detectedTagId + ")");
            telemetry.addData("Dist to Goal (in)", String.format("%.1f", calculatedDistanceInches));
            telemetry.addData("Target RPM", String.format("%.0f", (targetShooterVelocity * 60.0) / SHOOTER_TICKS_PER_REV));
            telemetry.addData("Shooter L / R RPM", String.format("%.0f / %.0f",
                    (Shooterl.getVelocity() * 60.0) / SHOOTER_TICKS_PER_REV,
                    (Shooterr.getVelocity() * 60.0) / SHOOTER_TICKS_PER_REV));
            telemetry.addData("Arm Est. Angle", currentEstimatedAngle);
            telemetry.update();
        }

        limelight.stop();
    }

    private void updateVisionAndHeading() {
        double rawImuYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        LLResult result = limelight.getLatestResult();

        hasValidGoalTag = false;
        detectedTagId = -1;

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double llFieldYaw = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

                if (isRedAlliance) {
                    llFieldYaw = normalizeAngle(llFieldYaw + 180.0);
                }

                double predictedHeading = normalizeAngle(rawImuYaw + headingOffset);
                double yawDifference = Math.abs(normalizeAngle(llFieldYaw - predictedHeading));

                if (yawDifference < MAX_YAW_JUMP_DEG) {
                    headingOffset = normalizeAngle(llFieldYaw - rawImuYaw);
                }
            }

            int targetId = isRedAlliance ? RED_GOAL_TAG_ID : BLUE_GOAL_TAG_ID;
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == targetId) {
                        hasValidGoalTag = true;
                        detectedTagId = targetId;
                        double ty = fiducial.getTargetYDegrees();
                        double angleToGoalRadians = Math.toRadians(LIMELIGHT_MOUNT_ANGLE_DEG + ty);
                        if (Math.abs(Math.tan(angleToGoalRadians)) > 0.001) {
                            calculatedDistanceInches = (TARGET_TAG_HEIGHT_IN - LIMELIGHT_LENS_HEIGHT_IN) / Math.tan(angleToGoalRadians);
                        }
                        break;
                    }
                }
            }
        }
        currentFieldHeadingDeg = normalizeAngle(rawImuYaw + headingOffset);
    }
    private double normalizeAngle(double angleDeg) {
        while (angleDeg > 180.0) angleDeg -= 360.0;
        while (angleDeg < -180.0) angleDeg += 360.0;
        return angleDeg;
    }
    private void armControlMethod() {
        if (isArmMoving) {
            if (armTimer.seconds() >= armTargetTime) {
                Arm.setPower(0.0);
                currentEstimatedAngle = targetAngle;
                isArmMoving = false;
            }
        } else {
            double requestedAngle = -1;
            if (gamepad1.y) {
                requestedAngle = 90.0;
            } else if (gamepad1.x) {
                requestedAngle = 45.0;
            } else if (gamepad1.a) {
                requestedAngle = 0.0;
            }
            if (requestedAngle != -1 && requestedAngle != currentEstimatedAngle) {
                targetAngle = requestedAngle;
                double angleDifference = targetAngle - currentEstimatedAngle;
                double direction = Math.signum(angleDifference);
                Arm.setPower(ARM_POWER * direction);
                armTargetTime = Math.abs(angleDifference) / DEGREES_PER_SECOND;
                armTimer.reset();
                isArmMoving = true;
            } else {
                Arm.setPower(0.0);
            }
        }
    }
    private void gateControlMethod() {
        if (gamepad1.right_trigger > 0.1) {
            Gate.setPosition(GATE_CLOCKWISE_POSITION);
        } else {
            Gate.setPosition(GATE_HOME_POSITION);
        }
    }
    private void shooterMethod(BasicPIDController pid) {
        if (gamepad1.b) {
            double desiredRPM;
            if (hasValidGoalTag && calculatedDistanceInches > 0) {
                desiredRPM = BASE_SHOOTER_RPM
                        + (calculatedDistanceInches * RPM_PER_INCH)
                        + (Math.pow(calculatedDistanceInches, 2) * RPM_PER_INCH_SQUARED);
                desiredRPM = Math.max(MIN_SHOOTER_RPM, Math.min(MAX_SHOOTER_RPM, desiredRPM));
            } else {
                desiredRPM = FALLBACK_SHOOTER_RPM;
            }
            targetShooterVelocity = (desiredRPM * SHOOTER_TICKS_PER_REV) / 60.0;
        } else if (gamepad1.left_trigger > 0.1) {
            targetShooterVelocity = -2500.0;
        } else {
            targetShooterVelocity = 0.0;
        }
        double currentVelL = Shooterl.getVelocity();
        double currentVelR = Shooterr.getVelocity();
        double powerL = pid.update(targetShooterVelocity, currentVelL) + (targetShooterVelocity * shooterF);
        double powerR = pid.update(targetShooterVelocity, currentVelR) + (targetShooterVelocity * shooterF);
        Shooterl.setPower(powerL);
        Shooterr.setPower(powerR);
    }
    private void intakeMethod() {
        if (gamepad1.right_trigger > 0.1) {
            irPower = 1.0;
            ilPower = 1.0;
        } else if (gamepad1.b) {
            irPower = -1.0;
            ilPower = -1.0;
        } else {
            irPower = 0.0;
            ilPower = 0.0;
        }
        Intakel.setPower(ilPower);
        Intaker.setPower(irPower);
    }
    public static class BasicPIDController {
        private double Kp, Ki, Kd;
        private double lastError = 0;
        private double integral = 0;
        private ElapsedTime timer = new ElapsedTime();
        public BasicPIDController(double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }
        public void setPID(double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }
        public double update(double reference, double state) {
            double error = reference - state;
            double dt = timer.seconds();
            timer.reset();
            integral += error * dt;
            double derivative = (dt != 0) ? (error - lastError) / dt : 0.0;
            lastError = error;
            return (Kp * error) + (Ki * integral) + (Kd * derivative);
        }
    }
}