package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.*;
public final class Intake extends SubsystemBase {
 private final MotorEx left,right; private double power;
 public Intake(HardwareMap hw){
  left=new MotorEx(hw,"intake1");right=new MotorEx(hw,"intake2");
  for(MotorEx m:new MotorEx[]{left,right}){m.setRunMode(Motor.RunMode.RawPower);m.setInverted(true);m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);}
 }
 public static double requested(boolean collect,boolean reverse){return reverse?-.9:collect?1:0;}
 public void set(double p){power=Double.isFinite(p)?Math.max(-1,Math.min(1,p)):0;}
 public void write(){left.set(power);right.set(power);}
 public void stop(){set(0);write();}
 public double getPower(){return power;}
}
