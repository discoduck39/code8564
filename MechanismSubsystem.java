package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Constants.MechanismConstants;


public class MechanismSubsystem extends SubsystemBase {

    // Motor for flipper//
    private SparkMax m_FlipperMOTOR3 = new SparkMax(MechanismConstants.kFlipper3CanId, MotorType.kBrushless);
    private SparkClosedLoopController m_FlipperMOTOR3Controller = m_FlipperMOTOR3.getClosedLoopController();
    private RelativeEncoder m_FlipperMOTOR3Encoder = m_FlipperMOTOR3.getEncoder();

    // Creates a new flipper Subsystem//
public MechanismSubsystem(){

m_FlipperMOTOR3.configure(
    Config.Mechanism.Flipper3Config,
    ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters
);
}

// get postion of flipper //

public double getFlipperMOTOR3(){
return m_FlipperMOTOR3Encoder.getPosition();
}

// reset flipper to 0//

public void ResetFlipperMOTOR3Encoder(){
    m_FlipperMOTOR3Encoder.setPosition(0);
}

// will move flipper to the specified setpoint using MaxMotion//

public void moveFlipperMOTOR3ToPostion(double Setpoint){
    m_FlipperMOTOR3Controller.setReference(Setpoint, ControlType.kMAXMotionPositionControl);
}



// stop flipper motor//

public void stopFlipperMOTOR3(){
m_FlipperMOTOR3.stopMotor();
}










    
}
