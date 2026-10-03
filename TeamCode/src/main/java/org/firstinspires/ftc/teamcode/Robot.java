package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.subsystems.*;
import java.util.List;
public final class Robot implements AutoCloseable {
 public Drive drive;public Intake intake;public Turret turret;public Hood hood;public Flywheel flywheel;
 public final VisionConfig visionConfig=new VisionConfig();
 private List<LynxModule> hubs;private Telemetry telemetry;
 public void init(HardwareMap hw,Telemetry t){
  telemetry=t;hubs=hw.getAll(LynxModule.class);
  for(LynxModule h:hubs)h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
  drive=new Drive(hw);intake=new Intake(hw);turret=new Turret(hw,visionConfig);hood=new Hood(hw);flywheel=new Flywheel(hw);
 }
 public void read(long now){for(LynxModule h:hubs)h.clearBulkCache();turret.read(now);flywheel.read(now);}
 public void write(){drive.update();intake.write();turret.write();hood.write();flywheel.write();}
 public void sendTelemetry(){telemetry.addData("Pose",drive.getPose());telemetry.addData("Turret feedback",turret.feedback.healthy);telemetry.addData("Flywheel RPM L/R","%.0f / %.0f",flywheel.getLeftRpm(),flywheel.getRightRpm());}
 public void close(){
  CommandScheduler.getInstance().cancelAll();
  try{if(intake!=null)intake.stop();if(flywheel!=null)flywheel.stop();if(drive!=null)drive.stop();if(turret!=null)turret.hold();if(hood!=null)hood.stow();}
  finally{CommandScheduler.getInstance().reset();if(hubs!=null)for(LynxModule h:hubs)h.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);}
 }
}
