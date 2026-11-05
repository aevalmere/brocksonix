package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.agateflow.DriveModel;
import org.firstinspires.ftc.teamcode.agateflow.TestRobot;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.subsystems.Door;
import org.firstinspires.ftc.teamcode.robot.subsystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Angles;
import org.horizon36596.simloop.config.ContinuousRotationSimConfig;
import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.config.PositionalServoSimConfig;
import org.horizon36596.simloop.fakehardware.FakeAnalogInput;
import org.horizon36596.simloop.fakehardware.FakeCRServo;
import org.horizon36596.simloop.fakehardware.FakeDigitalChannel;
import org.horizon36596.simloop.fakehardware.FakeGoBildaPinpoint;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.fakehardware.FakeServo;
import org.horizon36596.simloop.fakehardware.FakeVoltageSensor;
import org.horizon36596.simloop.plant.MecanumDrivePlant;
import org.horizon36596.simloop.plant.Mechanism1DofPlant;
import org.horizon36596.simloop.plant.PositionalServoPlant;

/**
 * Our robot in SimLoop: a fake under every configuration name Robot asks for, and a
 * plant behind each mechanism the autos use. Robot, Pedro and the Zenith runtime run
 * on top unchanged and never learn they are simulated.
 *
 * - Drive: the four motors under Pedro's real Mecanum, a MecanumDrivePlant, and a
 *   Pinpoint fed from where the plant really is. Speeds, lag and grip are TestRobot's
 *   (DriveModel's guesses), the same robot the AgateFlow tests drive.
 * - Shooter: FlywheelPlant. Turret: a continuous-rotation plant with the ELC encoder's
 *   voltage worked out from its angle. Door and flower intake: servo plants.
 * - Storage: RailPlant moves the balls with the intake and transfer motors, lets them
 *   through the door to the wheel and drives the three beams.
 * - Not here: the coax hood (the robot runs without it, HardwareNames.HOOD_SERVO), the
 *   field and anything to bump into, and battery sag (the voltage stays at VOLTS).
 *
 * The sim takes the robot as wired the way the code says: motor directions from
 * Constants, the REVERSED flags, the beam polarity and the turret encoder settings.
 * It checks the code's logic and timing, not the wiring.
 *
 * Poses are in Zenith's frame, (0, 0) at the field center (ZenithFrame): the plant
 * keeps the robot inside +-half the field around its origin. Same axes and headings
 * as our Pedro field, which is the same picture moved by Field.SIZE / 2.
 *
 * Built outside the OpMode's class loader (SimLoop loads our classes again for each
 * run), so it reads the code's constants as written, not as Panels might change them.
 * It talks to the run only through the hardware map and truth (see ZenithAutoHeadlessTest).
 */
public final class RobotSim {
    /** The battery voltage the whole run sees: TestRobot's, which the AgateFlow tests drive at. */
    static final double VOLTS = TestRobot.VOLTS;

    // ---- Drivetrain (ROBOT.md: goBILDA 6000 RPM bare motors, 10.5:1, 104 mm mecanum wheels) ----
    /** SPEC: 28 counts per motor turn, 10.5 motor turns per wheel turn, 104 mm wheels. */
    static final double DRIVE_TICKS_PER_INCH = 28 * 10.5 / (Math.PI * 104 / 25.4);
    // TODO(13): DriveModelTuner: measured top speeds and time constant. Until then TestRobot's, so the sim and the AgateFlow tests agree.
    /** PLACEHOLDER: top speeds with the robot's weight on the wheels, in/s (TestRobot.FORWARD and STRAFE). */
    static final double FORWARD_SPEED = TestRobot.FORWARD, STRAFE_SPEED = TestRobot.STRAFE;
    /** PLACEHOLDER: seconds for a wheel to close 63% of the gap to a new power (TestRobot.TAU_F). */
    static final double DRIVE_TIME_CONSTANT = TestRobot.TAU_F;
    // TODO(13): Put in the real track width and wheel base (wheel center to wheel center, ROBOT.md) once DriveModelTuner has the turn speed.
    /** PLACEHOLDER: half the track width plus half the wheel base, in., picked so full turn spins at TestRobot.TURN rad/s. */
    static final double HALF_TRACK_PLUS_HALF_BASE = FORWARD_SPEED / TestRobot.TURN;

    /** SPEC: a bare goBILDA 6000 RPM motor (shooter, intake, transfer) at free speed, 28 counts per turn. */
    static final double BARE_MOTOR_FREE_TICKS_PER_SECOND = 6000 * 28 / 60.0;

    // ---- Turret (ROBOT.md: an Axon in CR mode, turret turns twice per servo turn) ----
    // TODO(5): Turret Test, manual mode, full stick: time a few turns and put the turret's speed here, deg/s.
    /** PLACEHOLDER: turret speed at full servo power, deg/s. */
    static final double TURRET_SPEED = 720;
    // TODO(5): Same test, film a start from rest: seconds to reach 63% of full speed.
    /** PLACEHOLDER: seconds for the turret to close 63% of the gap to a new speed. */
    static final double TURRET_TIME_CONSTANT = 0.08;
    /** SPEC: the hub's analog port full scale, volts. */
    static final double ANALOG_MAX_VOLTS = 3.3;

    // ---- Servos ----
    // TODO(4): Film the door swinging closed to open: put its speed here, servo position per second.
    /** PLACEHOLDER: door servo speed, position units per second (0.3 of travel in 0.12 s). */
    static final double DOOR_SPEED = 2.5;
    /** A ball gets past the door once it is this far from closed toward open. */
    static final double DOOR_OPEN_ENOUGH = 0.8;
    // TODO(14): Film the flower intake coming up: its servo speed, position units per second.
    /** PLACEHOLDER: flower intake servo speed, position units per second. */
    static final double FLOWER_SPEED = 1.5;
    /** PLACEHOLDER: seconds for a servo horn to catch up to its speed. */
    static final double SERVO_TIME_CONSTANT = 0.03;

    // ---- Rail motors: bare 6000 RPM motors, encoders not read ----
    // TODO(3): Storage Test: how long the rail takes to reach full speed from rest, s.
    /** PLACEHOLDER: seconds for the intake and transfer to close 63% of the gap to a new power. */
    static final double RAIL_MOTOR_TIME_CONSTANT = 0.05;

    public final FakeHardwareMap hardwareMap = new FakeHardwareMap();

    final FakeMotor frontLeft, frontRight, backLeft, backRight;
    final LockstepPinpoint pinpoint = new LockstepPinpoint();
    final MecanumDrivePlant drivePlant;

    // The motors' own lag (50/s) goes unused: FlywheelPlant tells them how fast the wheel turns.
    final FakeMotor shooterLeft = new FakeMotor(BARE_MOTOR_FREE_TICKS_PER_SECOND, 50, 28);
    final FakeMotor shooterRight = new FakeMotor(BARE_MOTOR_FREE_TICKS_PER_SECOND, 50, 28);
    final FlywheelPlant flywheel;

    final FakeCRServo turretServo = new FakeCRServo();
    final FakeAnalogInput turretEncoder = new FakeAnalogInput(0, ANALOG_MAX_VOLTS);
    final Mechanism1DofPlant turretPlant;

    final FakeServo doorServo = new FakeServo();
    final PositionalServoPlant doorPlant;
    final FakeServo flowerLeftServo = new FakeServo();
    final FakeServo flowerRightServo = new FakeServo();
    final PositionalServoPlant flowerLeftPlant, flowerRightPlant;

    final FakeMotor intake = new FakeMotor(BARE_MOTOR_FREE_TICKS_PER_SECOND, 1 / RAIL_MOTOR_TIME_CONSTANT, 28);
    final FakeMotor transfer = new FakeMotor(BARE_MOTOR_FREE_TICKS_PER_SECOND, 1 / RAIL_MOTOR_TIME_CONSTANT, 28);
    final FakeDigitalChannel beamSlot1 = new FakeDigitalChannel();
    final FakeDigitalChannel beamSlot2 = new FakeDigitalChannel();
    final FakeDigitalChannel beamFull = new FakeDigitalChannel();
    final RailPlant rail = new RailPlant();

    /**
     * What really happened, for the trace, updated every step(): x, y and heading
     * (Zenith's frame, inches and radians), balls held, balls launched. A plain array,
     * so the run (which has its own copies of our classes) can read it.
     */
    public final double[] truth = new double[5];

    public RobotSim() {
        MecanumConfig drive = Constants.drivetrainConfig;
        frontLeft = driveMotor(drive.frontLeftName.get());
        frontRight = driveMotor(drive.frontRightName.get());
        backLeft = driveMotor(drive.backLeftName.get());
        backRight = driveMotor(drive.backRightName.get());
        final double[] mounting = {
                sign(drive.frontLeftDirection.get()), sign(drive.frontRightDirection.get()),
                sign(drive.backLeftDirection.get()), sign(drive.backRightDirection.get())};
        drivePlant = new MecanumDrivePlant(new FakeMotor[] {frontLeft, frontRight, backLeft, backRight},
                new DrivetrainSimConfig() {
                    @Override public double trackWidth() { return HALF_TRACK_PLUS_HALF_BASE; }
                    @Override public double wheelBase() { return HALF_TRACK_PLUS_HALF_BASE; }
                    @Override public double wheelRadius() { return 104 / 25.4 / 2; }
                    @Override public double ticksPerInch() { return DRIVE_TICKS_PER_INCH; }
                    @Override public double maxVelocityTicksPerSecond() { return FORWARD_SPEED * DRIVE_TICKS_PER_INCH; }
                    @Override public double maxLateralVelocityTicksPerSecond() { return STRAFE_SPEED * DRIVE_TICKS_PER_INCH; }
                    @Override public double maxAccel() { return 1 / DRIVE_TIME_CONSTANT; }
                    // Grip: DriveModel's guess, the most the floor can speed the robot up or slow it down.
                    @Override public double maxForwardAccelerationInchesPerSecondSquared() { return DriveModel.TRACTION_ACCEL; }
                    @Override public double maxLateralAccelerationInchesPerSecondSquared() { return DriveModel.TRACTION_ACCEL; }
                    // The motors on the robot are mounted the way Constants' directions say.
                    @Override public double[] wheelMountingSigns() { return mounting.clone(); }
                    @Override public double fieldHalfWidth() { return Field.SIZE / 2; }
                    @Override public double fieldHalfHeight() { return Field.SIZE / 2; }
                });

        PinpointConfig odometry = Constants.localizerConfig;
        // The pods sit where Constants says they do (ROBOT.md's CAD numbers), goBILDA 4-bar pods.
        pinpoint.setPhysicalPodMount(
                odometry.offsetUnits.get().toMm(odometry.xPodOffset.get()),
                odometry.offsetUnits.get().toMm(odometry.yPodOffset.get()),
                odometry.xPodDirection.get(), odometry.yPodDirection.get(),
                FakeGoBildaPinpoint.DEFAULT_TICKS_PER_MM);
        hardwareMap.register(odometry.name.get(), pinpoint);

        hardwareMap.register(HardwareNames.SHOOTER_LEFT, shooterLeft);
        hardwareMap.register(HardwareNames.SHOOTER_RIGHT, shooterRight);
        flywheel = new FlywheelPlant(shooterLeft, Shooter.LEFT_REVERSED ? -1 : 1,
                shooterRight, Shooter.RIGHT_REVERSED ? -1 : 1);

        hardwareMap.register(HardwareNames.TURRET_SERVO, turretServo);
        hardwareMap.register(HardwareNames.TURRET_ENCODER, turretEncoder);
        turretPlant = new Mechanism1DofPlant(turretServo, new ContinuousRotationSimConfig() {
            @Override public double timeConstant() { return TURRET_TIME_CONSTANT; }
            @Override public double maxSpeed() { return TURRET_SPEED; }
        });

        hardwareMap.register(HardwareNames.DOOR_SERVO, doorServo);
        doorPlant = new PositionalServoPlant(doorServo, servo(DOOR_SPEED));
        doorPlant.setPosition(Door.CLOSED_POSITION);
        hardwareMap.register(HardwareNames.FLOWER_LEFT_SERVO, flowerLeftServo);
        hardwareMap.register(HardwareNames.FLOWER_RIGHT_SERVO, flowerRightServo);
        flowerLeftPlant = new PositionalServoPlant(flowerLeftServo, servo(FLOWER_SPEED));
        flowerRightPlant = new PositionalServoPlant(flowerRightServo, servo(FLOWER_SPEED));
        flowerLeftPlant.setPosition(FlowerIntake.UP_POSITION);
        flowerRightPlant.setPosition(FlowerIntake.RIGHT_REVERSED ? 1 - FlowerIntake.UP_POSITION : FlowerIntake.UP_POSITION);

        hardwareMap.register(HardwareNames.INTAKE_MOTOR, intake);
        hardwareMap.register(HardwareNames.TRANSFER_MOTOR, transfer);
        hardwareMap.register(HardwareNames.BEAM_SLOT_1, beamSlot1);
        hardwareMap.register(HardwareNames.BEAM_SLOT_2, beamSlot2);
        hardwareMap.register(HardwareNames.BEAM_FULL, beamFull);

        hardwareMap.register("Control Hub", new FakeVoltageSensor(VOLTS));
        hardwareMap.register("Control Hub", new FakeLynxModule());

        place(0, 0, 0);
        readSensors();
    }

    private FakeMotor driveMotor(String name) {
        FakeMotor motor = new FakeMotor(FORWARD_SPEED * DRIVE_TICKS_PER_INCH, 1 / DRIVE_TIME_CONSTANT, 28 * 10.5);
        hardwareMap.register(name, motor);
        return motor;
    }

    private static double sign(DcMotorSimple.Direction direction) {
        return direction == DcMotorSimple.Direction.REVERSE ? -1 : 1;
    }

    /** A servo whose position is its command (0 to 1), moving at speed position units per second. */
    private static PositionalServoSimConfig servo(final double speed) {
        return new PositionalServoSimConfig() {
            @Override public double timeConstant() { return SERVO_TIME_CONSTANT; }
            @Override public double maxSpeed() { return speed; }
            @Override public double minPosition() { return 0; }
            @Override public double maxPosition() { return 1; }
            @Override public double positionAtCommandZero() { return 0; }
            @Override public double positionAtCommandOne() { return 1; }
        };
    }

    /** Puts the robot here, standing still, in Zenith's frame. */
    public void place(double x, double y, double heading) {
        drivePlant.setPose(x, y, heading);
        pinpoint.attachChassisAt(x * 25.4, y * 25.4, heading);
        readSensors();
    }

    /** Puts count balls in storage, slot 1 first, as the robot is loaded before a match. */
    public void preload(int count) {
        rail.load(count);
        readSensors();
    }

    /** One tick of the world, after the robot code wrote its powers (OpModeRunner's eachTick). */
    public void step(double dt) {
        drivePlant.update(dt);
        pinpoint.moveChassisTo(drivePlant.getX() * 25.4, drivePlant.getY() * 25.4, drivePlant.getHeading());
        pinpoint.advance(dt);

        turretPlant.update(dt);
        doorPlant.update(dt);
        flowerLeftPlant.update(dt);
        flowerRightPlant.update(dt);

        // The motors stay FORWARD in the code, so a REVERSED flag says the motor is mounted the other way.
        double transferSpeed = (Transfer.REVERSED ? -1 : 1) * transfer.getSpeed() * RailPlant.BALL_SPEED;
        double intakeSpeed = (Intake.REVERSED ? -1 : 1) * intake.getSpeed() * RailPlant.BALL_SPEED;
        double doorTravel = (doorPlant.getPosition() - Door.CLOSED_POSITION) / (Door.OPEN_POSITION - Door.CLOSED_POSITION);
        int launched = rail.update(dt, transferSpeed, intakeSpeed, doorTravel >= DOOR_OPEN_ENOUGH, flywheel.canLaunch());
        flywheel.update(dt, VOLTS, launched);

        readSensors();
    }

    /** Sets what the turret encoder and the beams read, and truth, from the plants. */
    private void readSensors() {
        double raw = turretPlant.getPosition() + Turret.ENCODER_ZERO_DEG;
        if (Turret.ENCODER_REVERSED) raw = -raw;
        double turn = ((raw % 360) + 360) % 360 / 360;
        turretEncoder.setVoltage(Turret.ENCODER_MIN_VOLTS + turn * (Turret.ENCODER_FULL_TURN_VOLTS - Turret.ENCODER_MIN_VOLTS));

        beam(beamSlot1, RailPlant.SLOT_1);
        beam(beamSlot2, RailPlant.SLOT_2);
        beam(beamFull, RailPlant.SLOT_4);

        truth[0] = drivePlant.getX();
        truth[1] = drivePlant.getY();
        truth[2] = Angles.wrapRadians(drivePlant.getHeading());
        truth[3] = rail.held();
        truth[4] = rail.launched();
    }

    private void beam(FakeDigitalChannel channel, double where) {
        boolean blocked = rail.blocked(where);
        channel.setSimulatedInputState(Storage.BLOCKED_READS_LOW != blocked);
    }

    public int held() {
        return rail.held();
    }

    public int launched() {
        return rail.launched();
    }

    public int spilled() {
        return rail.spilled();
    }
}
