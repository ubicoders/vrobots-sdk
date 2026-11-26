//! Actuator reports decoded through the C library's decoders, from reports
//! built here: the layouts, the "decode by type, never by count" trap, and the
//! names the library gives the codes. No simulator needed.

use vrobots_sdk::{
    forklift_actuator, forklift_channel, palletrobot_channel, rail_arm, table_arm, turret_actuator,
    Actuator, ActuatorKind, ArmContact, ForkliftActuators, PalletRobotActuators, RailArmActuators,
    TableArmActuators, TurretActuators,
};

/// A report with only `measured` filled in.
fn measured(values: &[f64]) -> Actuator {
    let mut report = Actuator::default();
    report.measured = values.to_vec();
    report
}

/// A report with `normalized` and `measured` filled in.
fn report(normalized: &[f64], values: &[f64]) -> Actuator {
    let mut report = measured(values);
    report.normalized = normalized.to_vec();
    report
}

#[test]
fn the_layouts_are_the_documented_numbers() {
    assert_eq!(
        (
            forklift_channel::STEER,
            forklift_channel::THROTTLE,
            forklift_channel::LIFT,
            forklift_channel::FORK_WIDTH,
            forklift_channel::BRAKE
        ),
        (0, 1, 2, 3, 4)
    );
    // The report leads with the drive, unlike the command.
    assert_eq!(
        (
            forklift_actuator::DRIVE,
            forklift_actuator::STEERING,
            forklift_actuator::COUNT
        ),
        (0, 1, 4)
    );
    assert_eq!(
        (palletrobot_channel::PAD_LIFT, palletrobot_channel::COUNT),
        (4, 6)
    );
    assert_eq!((rail_arm::JOINT_COUNT, rail_arm::ACTUATOR_COUNT), (7, 9));
    assert_eq!((rail_arm::RAIL, rail_arm::A6, rail_arm::VACUUM), (0, 6, 7));
    assert_eq!((table_arm::JOINT_COUNT, table_arm::ACTUATOR_COUNT), (6, 9));
    assert_eq!((table_arm::A1, table_arm::HOLD), (0, 8));
    assert_eq!(table_arm::GRIPPER_STROKE_M, 0.085);
    assert_eq!(
        (
            turret_actuator::HEADING,
            turret_actuator::LASER,
            turret_actuator::COUNT
        ),
        (0, 2, 3)
    );
    assert_eq!(
        forklift_actuator::KINDS,
        [
            ActuatorKind::Wheel,
            ActuatorKind::Servo,
            ActuatorKind::Linear,
            ActuatorKind::Linear
        ]
    );
}

#[test]
fn a_forklift_report_decodes_by_its_indices() {
    assert!(ForkliftActuators::from_actuator(&Actuator::default()).is_none());
    assert!(ForkliftActuators::from_actuator(&measured(&[1.0, 2.0, 3.0])).is_none());

    let r = measured(&[2.5, -0.1, 0.448, 0.454]);
    let a = ForkliftActuators::from_actuator(&r).expect("four entries");
    assert_eq!(a.drive_speed, r.measured[forklift_actuator::DRIVE]);
    assert_eq!(a.steering_angle, r.measured[forklift_actuator::STEERING]);
    assert_eq!(a.lift_height, r.measured[forklift_actuator::LIFT]);
    assert_eq!(a.fork_width, r.measured[forklift_actuator::FORK_WIDTH]);
}

#[test]
fn a_pallet_robot_report_decodes_by_its_channels() {
    let r = measured(&[5.6, -5.6, -5.6, 5.6, 0.06, -0.5]);
    let a = PalletRobotActuators::from_actuator(&r).expect("six entries");
    assert_eq!(a.wheel_speeds, [5.6, -5.6, -5.6, 5.6]);
    assert_eq!(a.wheel_speeds[palletrobot_channel::FR], -5.6);
    assert_eq!(a.pad_height, 0.06);
    assert_eq!(a.pad_angle, -0.5);
    // A forklift's report is too short to be a pallet robot's.
    assert!(PalletRobotActuators::from_actuator(&measured(&[1.0; 4])).is_none());
}

#[test]
fn the_two_arms_decode_their_own_layouts() {
    // A rail arm with the vacuum on and 12.5 kg held, stopped by its tool.
    let mut normalized = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 1.0, 0.0];
    let mut values = [4.0, 0.0, -1.0, 1.5, 0.0, 1.0, 0.0, 12.5, 1.0];
    let rail = RailArmActuators::from_actuator(&report(&normalized, &values)).expect("nine");
    assert_eq!(rail.joints, [4.0, 0.0, -1.0, 1.5, 0.0, 1.0, 0.0]);
    assert_eq!(rail.joints_normalized[rail_arm::RAIL], 0.1);
    assert!(rail.vacuum_on && rail.holding);
    assert_eq!(rail.held_mass_kg, 12.5);
    assert_eq!(rail.contact, ArmContact::ToolContact);

    // A table arm at rest, commanded shut on a 50 g sphere 59.7 mm wide.
    normalized = [0.0, -0.1, 0.25, 0.2, 0.25, 0.1, 0.0, 0.0, 1.0];
    values = [0.0, -0.6, 1.6, 1.3, 1.6, 0.8, 0.0597, 0.0, 0.05];
    let table_report = report(&normalized, &values);
    let table = TableArmActuators::from_actuator(&table_report).expect("nine");
    assert!(table.holding);
    assert_eq!(table.held_mass_kg, 0.05);
    assert_eq!(table.gripper_command_m, 0.0);
    assert_eq!(table.gripper_opening_m, 0.0597);
    assert_eq!(table.contact, ArmContact::NoContact);

    // Decode by type, never by count: the same nine entries read as a rail
    // arm's give wrong answers, not an error.
    let wrong = RailArmActuators::from_actuator(&table_report).expect("nine");
    assert_eq!(wrong.joints[6], 0.0597);
    assert_eq!(wrong.contact, ArmContact::Unknown);

    // Eight entries are no arm's report.
    assert!(TableArmActuators::from_actuator(&report(&[0.0; 8], &[0.0; 8])).is_none());
}

#[test]
fn a_turret_report_decodes_joints_and_laser() {
    let r = report(&[0.5, -0.25, 1.0], &[0.3, 0.1, 42.0]);
    let t = TurretActuators::from_actuator(&r).expect("three entries");
    assert_eq!((t.heading, t.pitch), (0.3, 0.1));
    assert_eq!(
        (t.heading_rate_fraction, t.pitch_rate_fraction),
        (0.5, -0.25)
    );
    assert!(t.laser_on);
    assert_eq!(t.laser_range, 42.0);
    // `measured` alone is not enough: the rates and the laser latch live in
    // `normalized`.
    assert!(TurretActuators::from_actuator(&measured(&[0.3, 0.1, 42.0])).is_none());
}

#[test]
fn codes_carry_the_names_the_library_gives_them() {
    for (contact, code, name) in [
        (ArmContact::NoContact, 0, "no_contact"),
        (ArmContact::ToolContact, 1, "tool_contact"),
        (ArmContact::CollisionStop, 2, "collision_stop"),
        (ArmContact::Unknown, -1, "unknown"),
    ] {
        assert_eq!(contact.code(), code);
        assert_eq!(ArmContact::from_code(code), contact);
        assert_eq!(contact.name(), name);
    }
    for (kind, code, name) in [
        (ActuatorKind::Unspecified, 0, "unspecified"),
        (ActuatorKind::Rotor, 1, "rotor"),
        (ActuatorKind::Servo, 2, "servo"),
        (ActuatorKind::Wheel, 3, "wheel"),
        (ActuatorKind::Surface, 4, "surface"),
        (ActuatorKind::Linear, 5, "linear"),
        (ActuatorKind::Vacuum, 6, "vacuum"),
        (ActuatorKind::Laser, 7, "laser"),
        (ActuatorKind::Gripper, 8, "gripper"),
        (ActuatorKind::Contact, 9, "contact"),
        (ActuatorKind::Hold, 10, "hold"),
        (ActuatorKind::Unknown, -1, "unknown"),
    ] {
        assert_eq!(kind.code(), code);
        assert_eq!(ActuatorKind::from_code(code), kind);
        assert_eq!(kind.name(), name);
    }
    assert_eq!(ActuatorKind::from_code(42), ActuatorKind::Unknown);
}
