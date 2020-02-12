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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="EncoderTest", group="Auto")
public class encoderTest extends LinearOpMode {

    DcMotor FRD = null;
    DcMotor FLD = null;
    DcMotor BRD = null;
    DcMotor BLD = null;

    @Override
    public void runOpMode()
    {
        FRD = hardwareMap.get(DcMotor.class, "FRD");
        FLD = hardwareMap.get(DcMotor.class, "FLD");
        BRD = hardwareMap.get(DcMotor.class, "BRD");
        BLD = hardwareMap.get(DcMotor.class, "BLD");

        FRD.setDirection(DcMotorSimple.Direction.FORWARD);
        FLD.setDirection(DcMotorSimple.Direction.FORWARD);
        BRD.setDirection(DcMotorSimple.Direction.FORWARD);
        BLD.setDirection(DcMotorSimple.Direction.FORWARD);

        FRD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FRD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FRD.setTargetPosition(100);
        FLD.setTargetPosition(100);
        BRD.setTargetPosition(100);
        BLD.setTargetPosition(100);

        if(!(FRD.isBusy() && FLD.isBusy() && BRD.isBusy() && BLD.isBusy())
        {
            FRD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FLD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}