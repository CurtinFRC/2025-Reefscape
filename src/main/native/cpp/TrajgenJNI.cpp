#include <jni.h>
#include <wpi/array.h>
#include <wpi/jni_util.h>

#include "fmt/base.h"
#include "jni_md.h"

extern "C" {
/*
 * Class:     frc_robot_jni_ShooterTrajoptJNI
 * Method:    calculateTrajectory
 * Signature: ([D????)V
 */
JNIEXPORT void JNICALL
Java_org_curtinfrc_frc2025_auto_TrajgenJNI_helloWorld
  (JNIEnv* env, jclass)
{
  fmt::println("Hello world!");
}
}  // extern "C"
