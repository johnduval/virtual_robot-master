package teamcode;

import virtual_robot.controller.LinearOpMode;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.hardware.*;
import virtual_robot.util.navigation.DistanceUnit;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
public class MapMecanum extends LinearOpMode {
    DCMotor rearLeftMotor = null;       //m1
    DCMotor frontLeftMotor = null;      //m2
    DCMotor frontRightMotor = null;     //m3
    DCMotor rearRightMotor = null;      //m4

    DistanceSensor frontDistance = null;
    DistanceSensor leftDistance = null;
    DistanceSensor rightDistance = null;
    DistanceSensor backDistance = null;

    GyroSensor gyro = null;

    Servo backServo = null;
    ColorSensor colorSensor = null;

    boolean runOpMode = true;

    public int targetForwardDistance1 = 7100;
    public int targetStrafeRight1 = 2500;

    public void runOpMode(){
        rearLeftMotor = hardwareMap.dcMotor.get("back_left_motor");     //m1
        frontLeftMotor = hardwareMap.dcMotor.get("front_left_motor");   //m2
        frontRightMotor = hardwareMap.dcMotor.get("front_right_motor"); //m3
        rearRightMotor = hardwareMap.dcMotor.get("back_right_motor");   //m4

        rearLeftMotor.setDirection(DCMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DCMotor.Direction.REVERSE);
        rearLeftMotor.setMode(DCMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DCMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DCMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DCMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DCMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DCMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DCMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DCMotor.RunMode.RUN_USING_ENCODER);
        gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        backServo = hardwareMap.servo.get("back_servo");
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        gyro.init();
        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        telemetry.addData("Press Start When Ready","");
        telemetryData();
        waitForStart();
        while (runOpMode){
            while (frontLeftMotor.getCurrentPosition() <= targetForwardDistance1 && runOpMode) {
                setMotorPowerForward(1);
                telemetryData();
            }
            stopMotors();
            resetEncoders();
            while (frontLeftMotor.getCurrentPosition() <= targetStrafeRight1 && runOpMode) {
                setMotorPowerRotateRight(1);
//                setMotorPowerForward(.5);
                telemetry.addData("HELLO", "");
                telemetryData();
            }
            stopMotors();
            runOpMode = false;
        }
        stopMotors();
        while (!runOpMode) {
            telemetryData();
        }
    }

    public void resetEncoders () {
        rearLeftMotor.setMode(DCMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DCMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DCMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DCMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DCMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DCMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DCMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DCMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorPowerForward (double power) {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        rearRightMotor.setPower(power);
        rearLeftMotor.setPower(power);
    }

    public void setMotorPowerReverse (double power) {
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
        rearRightMotor.setPower(-power);
        rearLeftMotor.setPower(-power);
    }

    public void setMotorPowerStrafeRight (double power) {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        rearLeftMotor.setPower(power);
        rearRightMotor.setPower(-power);
    }

    public void setMotorPowerStrafeLeft (double power) {
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
        rearLeftMotor.setPower(-power);
        rearRightMotor.setPower(power);
    }

    public void setMotorPowerRotateLeft (double power) {
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
        rearLeftMotor.setPower(power);
        rearRightMotor.setPower(-power);
    }

    public void setMotorPowerRotateRight (double power) {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        rearLeftMotor.setPower(-power);
        rearRightMotor.setPower(power);
    }

    public void stopMotors () {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
    }

    public void telemetryData () {
        telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
        telemetry.addData("Heading"," %.1f", gyro.getHeading());
        telemetry.addData("Front Distance", " %.1f", frontDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Encoders"," %d %d %d %d", rearLeftMotor.getCurrentPosition(), frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), rearRightMotor.getCurrentPosition());
        telemetry.update();
    }

    public void originalMecanumDriveCode () {
        double px = gamePad1.left_stick_x;
        if (Math.abs(px) < 0.05) px = 0;
        double py = -gamePad1.left_stick_y;
        if (Math.abs(py) < 0.05) py = 0;
        double pa = -gamePad1.right_stick_x;
        if (Math.abs(pa) < 0.05) pa = 0;
        double p1 = -px + py - pa;
        double p2 = px + py + -pa;
        double p3 = -px + py + pa;
        double p4 = px + py + pa;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        rearLeftMotor.setPower(p1);
        frontLeftMotor.setPower(p2);
        frontRightMotor.setPower(p3);
        rearRightMotor.setPower(p4);
    }
}
