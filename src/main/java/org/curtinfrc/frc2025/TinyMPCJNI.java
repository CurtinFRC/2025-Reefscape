package org.curtinfrc.frc2025;

import java.io.IOException;

import edu.wpi.first.hal.JNIWrapper;
import edu.wpi.first.math.jni.WPIMathJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.RuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;

public class TinyMPCJNI {
  static {
    WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
    JNIWrapper.Helper.setExtractOnStaticLoad(false);
    WPIMathJNI.Helper.setExtractOnStaticLoad(false);    try {
      CombinedRuntimeLoader.loadLibraries(
        TinyMPCJNI.class, "wpiutiljni", "wpimathjni", "wpiHaljni");      
      RuntimeLoader.loadLibrary("TinyMPCJNI");
    } catch (IOException e) {
      e.printStackTrace();
      System.exit(1);
    }  
  }

  // Initialize the TinyMPC solver
  public native void test();
  public native void initialize(int nStates, int nInputs, int horizon, float rho,
                                float[] AdynArr, float[] BdynArr, float[] QArr, float[] RArr);
  public native void setInitialState(float[] initialStateArr);
  public native void setReferenceTrajectory(float[] referenceTrajectoryArr, int horizon);
  public native float[] solveStep();
  public native void cleanup();
}
