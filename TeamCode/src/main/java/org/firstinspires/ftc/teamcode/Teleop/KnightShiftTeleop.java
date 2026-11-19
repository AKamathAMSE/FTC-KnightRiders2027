package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "KnightShiftTeleop")
public class KnightShiftTeleop extends LinearOpMode {
    private DcMotorEx LB, LF, RB, RF;
    private Servo FS;
    private DcMotorEx Shooterl, Shooterr;
    private DcMotorEx Intakel, Intaker;
    private Servo Arm;

    double lbPower, lfPower, rbPower, rfPower;
    double slPower, srPower, ilPower, irPower;

    public static double ARM_POS_1 = 0.482;
    public static double ARM_POS_2 = 0.4928;
    public static double ARM_POS_3 = 0.5036;
    public static double armTarget = 0.0;

    // PID Coefficients for Shooter (Tune these in FTC Dashboard)
    public static double shooterP = 0.001;
    public static double shooterI = 0.0;
    public static double shooterD = 0.0;
    public static double shooterF = 0.0; // Feedforward term

    private double targetShooterVelocity = 0.0;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        LB = hardwareMap.get(DcMotorEx.class, "LB");
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        RB = hardwareMap.get(DcMotorEx.class, "RB");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        Arm = hardwareMap.get(Servo.class, "Arm");
        Intakel = hardwareMap.get(DcMotorEx.class, "Intakel");
        Intaker = hardwareMap.get(DcMotorEx.class, "Intaker");
        Shooterr = hardwareMap.get(DcMotorEx.class, "Shooterr");
        Shooterl = hardwareMap.get(DcMotorEx.class, "Shooterl");

        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.REVERSE);
        Intakel.setDirection(DcMotor.Direction.FORWARD);
        Intaker.setDirection(DcMotor.Direction.REVERSE);
        Shooterl.setDirection(DcMotor.Direction.FORWARD);
        Shooterr.setDirection(DcMotor.Direction.REVERSE);
        Arm.setDirection(Servo.Direction.FORWARD);

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Initialize custom PID controller
        BasicPIDController shooterPID = new BasicPIDController(shooterP, shooterI, shooterD);

        while (opModeIsActive()) {
            // Update PID gains from Dashboard
            shooterPID.setPID(shooterP, shooterI, shooterD);

            intakeMethod();
            shooterMethod(shooterPID);
            feed();

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
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

            telemetry.addData("Run Time", runtime);
            telemetry.addData("Shooter Target Vel", targetShooterVelocity);
            telemetry.addData("Shooter Left Vel", Shooterl.getVelocity());
            telemetry.addData("Shooter Right Vel", Shooterr.getVelocity());
            telemetry.addData("Arm degree", Arm.getPosition());
            telemetry.addData("Intended Arm Degree", armTarget);
            telemetry.update();
        }
    }

    private void feed() {
        if (gamepad1.left_trigger > 1) {
            armTarget = 0.5;
        }
        else{
            armTarget = 0.0;
        }
        Arm.setPosition(armTarget);
    }

    private void shooterMethod(BasicPIDController pid) {
        if (gamepad1.b) {
            targetShooterVelocity = 2500.0;
        } else if (gamepad1.left_trigger > 1) {
            targetShooterVelocity = -2500.0;
        } else {
            targetShooterVelocity = 0.0;
        }

        // Calculate PID + Feedforward output based on current velocity
        double currentVelL = Shooterl.getVelocity();
        double currentVelR = Shooterr.getVelocity();

        double powerL = pid.update(targetShooterVelocity, currentVelL) + (targetShooterVelocity * shooterF);
        double powerR = pid.update(targetShooterVelocity, currentVelR) + (targetShooterVelocity * shooterF);

        Shooterl.setPower(powerL);
        Shooterr.setPower(powerR);
    }

    private void intakeMethod() {
        if (gamepad1.right_trigger > 0 ) {
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

    // Custom PID Controller helper class
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
