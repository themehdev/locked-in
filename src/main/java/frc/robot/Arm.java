package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;

public class Arm {
    public final SparkMax arm = new SparkMax(5, MotorType.kBrushed);

    public final Servo wrist = new Servo(9);

    public Arm() {
        SparkMaxConfig armconfig = new SparkMaxConfig();
        armconfig.encoder.countsPerRevolution(Constants.ENCODER_COUNTS_PER_REVOLUTION)
            .inverted(Constants.ENCODER_INVERTED);

        armconfig.closedLoop.
        p(Constants.ARM_kP)
        .i(Constants.ARM_kI)
        .d(Constants.ARM_kD);

        armconfig.closedLoop.feedForward.
        kS(Constants.ARM_kS)
        .kV(Constants.ARM_kV)
        .kA(Constants.ARM_kA);

        armconfig.closedLoop.maxMotion.
        cruiseVelocity(Constants.ARM_cruiseVelocity)
        .maxAcceleration(Constants.ARM_acceleration)
        .allowedProfileError(Constants.ARM_allowedProfileError);

        armconfig.smartCurrentLimit(9);

        armconfig.inverted(true);

        armconfig.idleMode(IdleMode.kBrake);

        arm.configure(armconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetArmEncoder();
    }

    private double getArmPosition() {
        return (arm.getEncoder().getPosition() * 360);
    }

    public void resetArmEncoder(){
        arm.getEncoder().setPosition(Constants.ARM_OFFSET/360.0);
    }

    private double clamp(double num, double min, double max){
        if(num <= min){
            return min;
        }else if(num >= max){
            return max;
        }else{
            return num;
        }
    }


    private void setArmPosition(double pos) {
        double position = pos;
        double multiplier = getArmPosition() < position ? 1.0 : 0.75;

        double posDelta = (position - getArmPosition());

        double to_volts_scaler = (12.0/(Constants.ARM_MAX_POWER_DEGS + Constants.ARM_HOLDING_VOLTAGE));

        arm.setVoltage(clamp((posDelta * multiplier * to_volts_scaler) + Constants.ARM_HOLDING_VOLTAGE, -11.5, 11.5));
        // if(Math.abs(getArmPosition() - position) > Constants.ARM_HOLDING_THRESHOLD) {
        //     arm.setVoltage(Constants.ARM_MOVING_VOLTAGE * multiplier);
        // } else if (Math.abs(getArmPosition() - position) > Constants.ARM_DONE_THRESHOLD) {
        //     arm.setVoltage(Constants.ARM_HOLDING_VOLTAGE * multiplier);
        // } else {
        //     arm.setVoltage(0.0);
        // }
    }

    public void setWrist(double degs){
        double wristPos = (degs - Constants.WRIST_OFFSET)/300.0;
        wrist.set(wristPos);
    }

    public void setPosition(Constants.POSITIONS position) {
        setWrist(position.wristPos);
        setArmPosition(position.armPos);

        System.out.println(position.name());
        System.out.println(position.armPos);
        System.out.println(getArmPosition());
        System.out.println(arm.getAppliedOutput());
        System.out.println(position.wristPos);
        System.out.println(wrist.get() * 300 + Constants.WRIST_OFFSET);
    }
}
