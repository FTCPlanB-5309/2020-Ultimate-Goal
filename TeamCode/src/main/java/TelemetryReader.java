import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "TelemetryReader")

public class TelemetryReader extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.voltage = getBatteryVoltage();
        telemetry.addData("voltage", robot.voltage);
        telemetry.update();
        waitForStart();
        telemetry.addData("Left Distance One", robot.leftDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Left Distance Two", robot.leftDistanceSensorTwo.getDistance(DistanceUnit.INCH));
        telemetry.addData("Front Distance", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Rear Distance", robot.rearDistanceSensor.getDistance(DistanceUnit.INCH));
//        telemetry.addData("Red", robot.colorSensor.red());
//        telemetry.addData("Blue", robot.colorSensor.blue());
//        telemetry.addData("Green", robot.colorSensor.green());
//        telemetry.addData("Alpha", robot.colorSensor.alpha());
        telemetry.update();

    }
    double getBatteryVoltage() {double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {result = Math.min(result, voltage);}
        }
        return result;
    }
}
