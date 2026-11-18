
package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Showcase")
public class Showcase extends LinearOpMode {

    private DcMotorEx LB, LF, RB, RF;
    private Servo Gate;
    private DcMotorEx Shooterl, Shooterr;
    private DcMotorEx Intakel, Intaker;
    private CRServo Arm;

    double lbPower, lfPower, rbPower, rfPower;
    double slPower, srPower, ilPower, irPower;

    // Arm variables
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

    // Gate servo positions
    public static double GATE_HOME_POSITION = 0.0;
    public static double GATE_CLOCKWISE_POSITION = 0.5;

    // Shooter PID coefficients
    public static double shooterP = 0.001;
    public static double shooterI = 0.0;
    public static double shooterD = 0.0;
    public static double shooterF = 0.0;

    private double targetShooterVelocity = 0.0;

    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();

        // Hardware mapping
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

        // Motor directions
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

        // Reset encoders
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Intakel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intaker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Shooterl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooterr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set run modes
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Intakel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intaker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Shooterl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooterr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set Gate to original position
        Gate.setPosition(GATE_HOME_POSITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Initialize shooter PID
        BasicPIDController shooterPID =
                new BasicPIDController(shooterP, shooterI, shooterD);

        while (opModeIsActive()) {

            shooterPID.setPID(shooterP, shooterI, shooterD);

            intakeMethod();
            shooterMethod(shooterPID);
            armControlMethod();
            gateControlMethod();

            // Mecanum drive
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            lbPower = forward - strafe + turn;
            lfPower = forward + strafe + turn;
            rbPower = forward + strafe - turn;
            rfPower = forward - strafe - turn;

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

            // Telemetry
            telemetry.addData("Run Time", runtime);
            telemetry.addData("Shooter Target Vel", targetShooterVelocity);
            telemetry.addData("Shooter Left Vel", Shooterl.getVelocity());
            telemetry.addData("Shooter Right Vel", Shooterr.getVelocity());
            telemetry.addData("Arm Est. Angle", currentEstimatedAngle);
            telemetry.addData("Arm Moving", isArmMoving);
            telemetry.addData("Gate Position", gamepad1.right_trigger > 0.1
                    ? GATE_CLOCKWISE_POSITION : GATE_HOME_POSITION);
            telemetry.update();
        }
    }

    // CR Servo arm control
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

            if (requestedAngle != -1 &&
                    requestedAngle != currentEstimatedAngle) {

                targetAngle = requestedAngle;

                double angleDifference =
                        targetAngle - currentEstimatedAngle;

                double direction = Math.signum(angleDifference);

                Arm.setPower(ARM_POWER * direction);

                armTargetTime =
                        Math.abs(angleDifference) / DEGREES_PER_SECOND;

                armTimer.reset();
                isArmMoving = true;

            } else {
                Arm.setPower(0.0);
            }
        }
    }

    // Standard servo Gate control
    private void gateControlMethod() {

        if (gamepad1.right_trigger > 0.1) {
            Gate.setPosition(GATE_CLOCKWISE_POSITION);
        } else {
            Gate.setPosition(GATE_HOME_POSITION);
        }
    }

    // Shooter control
    private void shooterMethod(BasicPIDController pid) {

        if (gamepad1.b) {
            targetShooterVelocity = 2500.0;
        } else if (gamepad1.left_trigger > 0.1) {
            targetShooterVelocity = -2500.0;
        }
        else
        {
            targetShooterVelocity = 0.0;
        }

        double currentVelL = Shooterl.getVelocity();
        double currentVelR = Shooterr.getVelocity();

        double powerL = pid.update(targetShooterVelocity, currentVelL)
                + (targetShooterVelocity * shooterF);

        double powerR = pid.update(targetShooterVelocity, currentVelR)
                + (targetShooterVelocity * shooterF);

        Shooterl.setPower(powerL);
        Shooterr.setPower(powerR);
    }

    // Intake control
    private void intakeMethod() {

        if (gamepad1.right_trigger > 0.1) {
            irPower = 1.0;
            ilPower = 1.0;
        }
        else if (gamepad1.b) {
            irPower = -1.0;
            ilPower = -1.0;
        }
        else {
            irPower = 0.0;
            ilPower = 0.0;
        }

        Intakel.setPower(ilPower);
        Intaker.setPower(irPower);
    }

    // Custom PID controller
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

            double derivative =
                    (dt != 0) ? (error - lastError) / dt : 0.0;

            lastError = error;

            return (Kp * error)
                    + (Ki * integral)
                    + (Kd * derivative);
        }
    }
}
