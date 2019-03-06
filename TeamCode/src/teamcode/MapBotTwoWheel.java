package teamcode;

import virtual_robot.controller.LinearOpMode;
import virtual_robot.hardware.ColorSensor;
import virtual_robot.hardware.DCMotor;
import virtual_robot.hardware.DistanceSensor;
import virtual_robot.hardware.GyroSensor;
import virtual_robot.util.navigation.DistanceUnit;

public class MapBotTwoWheel extends LinearOpMode {
    DCMotor leftMotor, rightMotor = null;
    GyroSensor gyro = null;
    int turn1 = -90;
    int turn2 = -180;
    int redThreshold = 50;
    int blueThreshold = 50;
    int redMinReading = 255;
    int blueMinReading = 255;
    int greenMinReading = 255;
    int targetEncoder1 = 7100;
    boolean runOpMode = true;
    ColorSensor colorSensor;
    DistanceSensor frontDistance;
    DistanceSensor leftDistance;
    DistanceSensor backDistance;
    DistanceSensor rightDistance;
    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");
        leftMotor.setDirection(DCMotor.Direction.REVERSE);
        rightMotor.setDirection(DCMotor.Direction.FORWARD);
        leftMotor.setMode(DCMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DCMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DCMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DCMotor.RunMode.RUN_USING_ENCODER);
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        gyro.init();
        telemetry.addData("Press START when ready","");
        telemetryData();
        waitForStart();
        while (runOpMode) {

            while (leftMotor.getCurrentPosition() < targetEncoder1 && runOpMode) {
                leftMotor.setPower(1);
                rightMotor.setPower(1);
                telemetryData();
            }
            stopMotors ();
            while (gyro.getHeading() >= turn1 && runOpMode) {
                leftMotor.setPower(.5);
                rightMotor.setPower(-.5);
                telemetryData();
            }
            while (gyro.getHeading() <= turn1 && runOpMode) {
                leftMotor.setPower(-.05);
                rightMotor.setPower(.05);
                telemetryData();
            }
            stopMotors();
            while (colorSensor.blue() <= blueThreshold) {
                telemetryData();
                leftMotor.setPower(.5);
                rightMotor.setPower(.5);
            }
            gyro.init();
            stopMotors();
            while (gyro.getHeading() >= turn2 && runOpMode) {
                leftMotor.setPower(.5);
                rightMotor.setPower(-.5);
                telemetryData();
            }
            while (gyro.getHeading() <= turn2 && runOpMode) {
                leftMotor.setPower(-.05);
                rightMotor.setPower(.05);
                telemetryData();
            }

            runOpMode = false;
        }
        stopMotors();
        while (!runOpMode) {
            telemetryData();
        }
    }

    public void stopMotors () {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void colorValues () {
        if (colorSensor.red() < redMinReading) {
            redMinReading = colorSensor.red();
        }
        if (colorSensor.blue() < blueMinReading) {
            blueMinReading = colorSensor.blue();
        }
        if (colorSensor.green() < greenMinReading) {
            greenMinReading = colorSensor.green();
        }
    }

    public void telemetryData () {
//        telemetry.addData("CURRENT LEFT: ", leftMotor.getCurrentPosition());
//        telemetry.addData("CURRENT RIGHT: ", rightMotor.getCurrentPosition());
        colorValues();
        telemetry.addData("Encoders","Left %d  Right %d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
        telemetry.addData("Heading"," %.1f", gyro.getHeading());
        telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
        telemetry.addData("RED MIN: ", redMinReading);
        telemetry.addData("BLUE MIN: ", blueMinReading);
        telemetry.addData("GREEN MIN: ", greenMinReading);
        telemetry.addData("Distance", " Fr %.1f  Lt %.1f  Rt %.1f  Bk %.1f  ",
                frontDistance.getDistance(DistanceUnit.CM), leftDistance.getDistance(DistanceUnit.CM),
                rightDistance.getDistance(DistanceUnit.CM), backDistance.getDistance(DistanceUnit.CM)
        );
        telemetry.update();
    }
}
