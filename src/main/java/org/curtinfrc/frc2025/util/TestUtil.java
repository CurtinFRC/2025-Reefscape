package org.curtinfrc.frc2025.util;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class TestUtil {
  static enum InputType {
    MOTOR,
    SENSOR;
  }

  static record MotorData(boolean alive) {}

  public abstract class Input<T> {
    String _path;

    public Input(String path) {
      this._path = path;
    }

    public String path() {
      return this._path;
    }

    public abstract T get();
  }

  public class DigitalSensor extends Input<Boolean> {
    private int _port;
    private DigitalInput _sensor;

    public DigitalSensor(int port, String name) {
      super("sensor/" + name);
      this._port = port;
      this._sensor = new DigitalInput(this._port);
    }

    public DigitalSensor(int port) {
      super("sensor/" + port);
      this._port = port;
      this._sensor = new DigitalInput(this._port);
    }

    public Boolean get() {
      return this._sensor.get();
    }
  }

  public class Motor extends Input<MotorData> {
    enum MotorType {
      TalonFX,
      SparkMax
    }

    private int _port;
    private MotorType type;
    private Optional<TalonFX> talon = Optional.empty();
    private Optional<SparkMax> spark = Optional.empty();

    public Motor(SparkMax sparkMax, String name) {
      super("motors/" + name);
      this.type = MotorType.SparkMax;
      this.spark = Optional.of(sparkMax);
    }

    public Motor(SparkMax sparkMax) {
      super("motors/" + sparkMax.getDeviceId());
      this.type = MotorType.SparkMax;
      this.spark = Optional.of(sparkMax);
    }

    public Motor(TalonFX talonFX, String name) {
      super("motors/" + name);
      this.type = MotorType.TalonFX;
      this.talon = Optional.of(talonFX);
    }

    public Motor(TalonFX talonFX) {
      super("motors/" + talonFX.getDeviceID());
      this.type = MotorType.TalonFX;
      this.talon = Optional.of(talonFX);
    }

    public MotorData get() {
      switch (this.type) {
        case TalonFX:
          return new MotorData(this.talon.get().isAlive());
        case SparkMax:
          return new MotorData(this.spark.get().getFirmwareVersion() != 0);
        default:
          return new MotorData(false);
      }
    }
  }

  private List<Input> inputs;
  private NetworkTableInstance inst;

  public TestUtil() {
    this.inst = NetworkTableInstance.getDefault();
    this.inputs = new ArrayList<Input>();
  }

  public void tick() {
    this.inputs.forEach(
        (input) -> {
          Object value = input.get();
          Object networkValue = getNetworkTableCompatibleValue(value);
          this.inst.getEntry("Tests/" + input.path()).setValue(networkValue);
        });
  }

  private Object getNetworkTableCompatibleValue(Object obj) {
    if (obj == null) {
      return null;
    }

    Class<?> clazz = obj.getClass();

    if (clazz.isPrimitive()
        || obj instanceof String
        || obj instanceof Boolean
        || obj instanceof Number) {
      return obj; // Directly compatible
    }

    if (clazz.isArray()) {
      Object[] array = (Object[]) obj;
      Object[] networkArray = new Object[array.length];
      for (int i = 0; i < array.length; i++) {
        networkArray[i] = getNetworkTableCompatibleValue(array[i]);
      }
      return networkArray;
    }

    if (obj instanceof Optional) {
      Optional<?> optional = (Optional<?>) obj;
      if (optional.isPresent()) {
        return getNetworkTableCompatibleValue(optional.get());
      } else {
        return null;
      }
    }

    // Check if it's a record or a regular object and recursively handle fields.
    if (clazz.isRecord() || !clazz.getPackageName().startsWith("java.")) {
      try {
        Field[] fields = clazz.getDeclaredFields();
        if (fields.length == 1 && fields[0].getType() == boolean.class) {
          try {
            return clazz.getMethod(fields[0].getName()).invoke(obj);
          } catch (Exception e) {
            return false;
          }
        }
        for (Field field : fields) {
          field.setAccessible(true);
          Object fieldValue = field.get(obj);
          Object networkFieldValue = getNetworkTableCompatibleValue(fieldValue);
          if (networkFieldValue != null) {
            return networkFieldValue;
          }
        }
        return null;
      } catch (IllegalAccessException e) {
        return null;
      }
    }

    return null; // Not compatible
  }

  public void registerCommand() {}

  public void addInput(Input input) {
    this.inputs.add(input);
  }
}
