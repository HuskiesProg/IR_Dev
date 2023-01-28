package frc.robot.subsystems;


import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
  private NetworkTable limelight = networkTableInstance.getTable("limelight");
  private NetworkTableEntry botpose = limelight.getEntry("botpose");
  private NetworkTableEntry stream = limelight.getEntry("stream");
  double[] result;
  double[] temp = { 0,0,0,0,0,0};

  public Limelight() {
    stream.setNumber(2);//Pour mettre l'image de la limelight en PiP
    
  }

  @Override
  public void periodic() {

    //Valeur complète du Pose 3D
    SmartDashboard.putString("Pose", getPos().toString());

    //L'angle du robot par rapport au AprilTag normalisé en Degree
    SmartDashboard.putNumber("ANgle Camera",Math.toDegrees( getPos().getRotation().getZ()));
  }

//Function qui retourne la position 3D du robot par rapport à un AprilTag
  public Pose3d getPos(){
    //default for getEntry

    result = botpose.getDoubleArray(temp);

    if(result.length == 0)
      result = temp;

    Translation3d tran3d = new Translation3d(result[0], result[1], result[2]);
    
    Rotation3d r3d = new Rotation3d(Math.toRadians(result[3]), Math.toRadians(result[4]), Math.toRadians(result[5]));
    Pose3d p3d = new Pose3d(tran3d, r3d);
    
    return p3d;
  } 
}