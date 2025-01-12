/*package frc.utils;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

public class SparkWrapper extends SparkMax {
    public SparkWrapper(int deviceId, SparkLowLevel.MotorType type) {
        super(deviceId, type);
    }

    @SuppressWarnings("deprecation")
    @Override
    public void setInverted(boolean isInverted) {
        
        for (int i = 0; i <= 5; i++) {
            super.setInverted(isInverted);
            if (super.getInverted() == isInverted) {
                break;
            }
        }
    }

    public void setPID(double pGain, double iGain, double dGain, double fGain){
        for (int i = 0; i <= 5; i++) {
            super.closedLoopController.pidf(pGain,iGain,dGain,fGain);
            if (this.sparkPID.getP() == gain) {
                break;
            }
        }
    }

    public SparkPIDWrapper getPIDWrapper() {
        return new SparkPIDWrapper(super.getClosedLoopController());
    }

}
    */
