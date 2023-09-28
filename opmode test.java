/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name = "Basic: Iterative OpMode", group = "Iterative OpMode")
@Disabled
public class BasicOpMode_Iterative extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private DcMotor motorArmLeft = null;
    private Dcmotor motorArmRight = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        motorArmLeft = hardwareMap.get(DcMotor.class, "arm_lef");
        motorArmRight = hardwareMap.get(DcMotor.class, "arm_right");

        motorArmLeft.setDirection(Dc.Motor.Direction.REVERSE);
        motorArmRight.setDirection(Dc.Motor.Direction.FOWARD);


        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();

        //wheels movement vars
        double leftPower;
        double rightPower;
        double drive;
        double turn;

        //arm movement vars
        double armPower;
    }

    @Override
    public void loop() {
        wheels_move();
        arm_move();
        att_telemetry();
    }

    public void arm_move() {
        armPower = gamepad1.right_stick_y;
        armPower = Range.clip(arm_power, -1.0, 1.0);
        motorArmLeft.setPower(armPower)
        motorArmRight.setPower(armPower)
    }

    public void wheels_move() {
        drive = -gamepad1.left_stick_y;
        turn = gamepad1.left_stick_x;
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void att_telemetry() {

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Wheels motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Arm Motors", "Arm power: " + armPower);
    }

    @Override
    public void stop() {
    }
}
