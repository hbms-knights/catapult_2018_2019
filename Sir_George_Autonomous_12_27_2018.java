package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Auto 12-10", group="Pushbot")

public class Sir_George_Autonomous_12_10_2018 extends LinearOpMode {
    
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private ElapsedTime runtime = new ElapsedTime();

    
    BNO055IMU imu;
    
    Orientation angles;
    
    @Override
    public void runOpMode() {
        leftDrive  = hardwareMap.get(DcMotor.class, "left_wheel");
        rightDrive = hardwareMap.get(DcMotor.class, "right_wheel");
        
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
       
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();
        runtime.reset();

        leftDrive.setPower(1);
        rightDrive.setPower(1);
        runtime.reset();
        
        // Head to Depot
        while (opModeIsActive() && (runtime.seconds() < 2.5)) {
        }  

        leftDrive.setPower(-0.5);
        rightDrive.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.1)) {      
        }

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        leftDrive.setPower(0.4);
        rightDrive.setPower(-0.4);
        
        // Turn towards crater
        while(opModeIsActive() && Math.abs(angles.firstAngle) < 55) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("angle ", Math.abs(angles.firstAngle));
            telemetry.update();
        }
        
        // Head to crater
        leftDrive.setPower(-1);
        rightDrive.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 4)) {      
        }
    } 
}
