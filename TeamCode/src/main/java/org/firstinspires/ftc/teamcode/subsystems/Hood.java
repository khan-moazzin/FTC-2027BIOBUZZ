package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import org.firstinspires.ftc.teamcode.config.MechanismConfig;
import org.firstinspires.ftc.teamcode.control.Angles;
public final class Hood extends SubsystemBase {
 private final ServoEx servo;private double position=MechanismConfig.hoodStow;private long changed;
 public Hood(HardwareMap hw){servo=new ServoEx(hw,"hood");changed=System.nanoTime();}
 public void setPosition(double p){if(!Double.isFinite(p))return;p=Angles.clamp(p,MechanismConfig.hoodMin,MechanismConfig.hoodMax);if(p!=position){position=p;changed=System.nanoTime();}}
 public void setAngle(double a){setPosition(MechanismConfig.hoodZero+a/MechanismConfig.hoodRadiansPerUnit);}
 public boolean ready(long now){return MechanismConfig.hoodCalibrated&&now-changed>MechanismConfig.hoodSettle*1e9;}
 public void write(){servo.set(position);}public void stow(){setPosition(MechanismConfig.hoodStow);write();}
 public double getPosition(){return position;}
}
