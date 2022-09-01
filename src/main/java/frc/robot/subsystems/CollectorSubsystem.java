package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CollectorSubsystem extends SubsystemBase {
    Constants constants = new Constants();
    WPI_TalonSRX frontCollector = new WPI_TalonSRX(constants.FRONT_COLLECTOR_CAN_ID);
    WPI_TalonSRX backCollector = new WPI_TalonSRX(constants.BACK_CoLLECTOR_CAN_ID);
    int inversion = 1;


    public CollectorSubsystem() {

    }

    public void periodic(){
        
    }

    public void setMotor(double speed, boolean inverted){
        if(inverted = true){
            inversion = -1;
        }
        frontCollector.set(speed * inversion);
        backCollector.set(speed * inversion);
    }


    public void doubleBall(){
        frontCollector.set(.9);
        backCollector.set(-.9);
    }

    
}
