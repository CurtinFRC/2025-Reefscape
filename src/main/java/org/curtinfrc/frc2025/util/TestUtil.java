package org.curtinfrc.frc2025.util;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

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
    private BooleanSupplier _get;

    public DigitalSensor(int port, String name) {
      super("sensor/" + name);
      DigitalInput in = new DigitalInput(port);
      this._get = () -> in.get();
    }

    public DigitalSensor(int port) {
      super("sensor/" + port);
      DigitalInput in = new DigitalInput(port);
      this._get = () -> in.get();
    }

    public DigitalSensor(SimBoolean sim, String name) {
      super("sensor/" + name);
      this._get = () -> sim.get();
    }

    public DigitalSensor(DigitalInput in, String name) {
      super("sensor/" + name);
      this._get = () -> in.get();
    }

    public Boolean get() {
      return _get.getAsBoolean();
    }
  }

  public class Motor extends Input<MotorData> {
    enum MotorType {
      TalonFX,
      SparkMax,
      Sim
    }

    private int _port;
    private MotorType type;
    private Optional<TalonFX> talon = Optional.empty();
    private Optional<SparkMax> spark = Optional.empty();
    private Optional<DCMotorSim> sim = Optional.empty();

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

    public Motor(DCMotorSim sim, String name) {
      super("motors/" + name);
      this.sim = Optional.of(sim);
      this.type = MotorType.Sim;
    }

    public MotorData get() {
      switch (this.type) {
        case TalonFX:
          return new MotorData(this.talon.get().isAlive());
        case SparkMax:
          return new MotorData(this.spark.get().getFirmwareVersion() != 0);
        case Sim:
          return new MotorData(true);
        default:
          return new MotorData(false);
      }
    }
  }

  public class Callable {
    private Command command;
    private String _name;

    public Callable(Command c, String name) {
      command = c;
      _name = name;
    }

    public Command get() {
      return command;
    }

    public String name() {
      return _name;
    }
  }

  private enum CommandStatus {
    kStarted,
    kStopped,
    kNone
  }

  private List<Input> inputs;
  private NetworkTableInstance inst;
  private List<Callable> commands;
  private Supplier<String> current_command;
  private CommandStatus status = CommandStatus.kNone;

  public TestUtil() {
    this.inst = NetworkTableInstance.getDefault();
    this.inputs = new ArrayList<Input>();
    this.commands = new ArrayList<Callable>();
    NetworkTableEntry entry = this.inst.getEntry("Tests/currentCommand");
    this.current_command = () -> entry.getString("NULL");
  }

  public void tick() {
    this.inputs.forEach(
        input -> {
          Object value = input.get();
          Map<String, Object> values = getNetworkTableCompatibleValues(value);

          values.forEach(
              (suffix, networkValue) -> {
                this.inst.getEntry("Tests/" + input.path() + suffix).setValue(networkValue);
              });
        });
    
    String current = this.current_command.get();

    d = false;
    this.commands.forEach((Callable c) -> {
      if (c.name() == current) {
        c.get().execute();
        this.status = CommandStatus.kStarted;
        d = true;
      } else {
        CommandScheduler.getInstance().cancel(c.get());
      }
    });

    if (!d) {
      this.status = CommandStatus.kNone;
    }
  }

  private Map<String, Object> getNetworkTableCompatibleValues(Object obj) {
    Map<String, Object> values = new HashMap<>();

    if (obj == null) {
      return values;
    }

    Class<?> clazz = obj.getClass();

    if (clazz.isPrimitive()
        || obj instanceof String
        || obj instanceof Boolean
        || obj instanceof Number) {
      values.put("", obj); // Store the main value under the base key
      return values;
    }

    if (clazz.isArray()) {
      Object[] array = (Object[]) obj;
      for (int i = 0; i < array.length; i++) {
        values.put("/" + i, getNetworkTableCompatibleValues(array[i]));
      }
      return values;
    }

    if (obj instanceof Optional) {
      Optional<?> optional = (Optional<?>) obj;
      if (optional.isPresent()) {
        return getNetworkTableCompatibleValues(optional.get());
      } else {
        return values;
      }
    }

    if (clazz.isRecord()) {
      try {
        Field[] fields = clazz.getDeclaredFields();
        for (Field field : fields) {
          field.setAccessible(true);
          Object fieldValue = field.get(obj);
          Map<String, Object> subValues = getNetworkTableCompatibleValues(fieldValue);
          for (Map.Entry<String, Object> entry : subValues.entrySet()) {
            values.put("/" + field.getName() + entry.getKey(), entry.getValue());
          }
        }
      } catch (IllegalAccessException e) {
        return values;
      }
    }

    return values;
  }

  public void registerCommand(Command c, String name) {
    this.commands.add(this.new Callable(c.andThen(() -> this.status = CommandStatus.kNone), name));
  }

  public void addInput(Input input) {
    this.inputs.add(input);
  }
}
