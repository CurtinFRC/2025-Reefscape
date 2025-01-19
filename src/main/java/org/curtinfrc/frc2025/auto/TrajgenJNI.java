package org.curtinfrc.frc2025.auto;

import edu.wpi.first.hal.JNIWrapper;
import edu.wpi.first.math.jni.WPIMathJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.RuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;
import java.io.IOException;

public class TrajgenJNI {
  static {
    WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
    JNIWrapper.Helper.setExtractOnStaticLoad(false);
    WPIMathJNI.Helper.setExtractOnStaticLoad(false);
    try {
      CombinedRuntimeLoader.loadLibraries(
          TrajgenJNI.class, "wpiutiljni", "wpimathjni", "wpiHaljni");
      RuntimeLoader.loadLibrary("TrajgenJNI");
    } catch (IOException e) {
      e.printStackTrace();
      System.exit(1);
    }
  }

  public static native void helloWorld();
}
