# The VRobots SDK Book

[Introduction](introduction.md)

---

# Tutorial

- [Getting started](ch01-getting-started/00-intro.md)
    - [Installing the SDK and the simulator](ch01-getting-started/01-install.md)
    - [First contact: is anything publishing?](ch01-getting-started/02-first-contact.md)
    - [Hello states](ch01-getting-started/03-hello-states.md)
    - [Hello control](ch01-getting-started/04-hello-control.md)
    - [Hello car](ch01-getting-started/05-hello-car.md)
    - [Hello image](ch01-getting-started/06-hello-image.md)
    - [Hello service](ch01-getting-started/07-hello-service.md)
    - [When nothing happens](ch01-getting-started/08-troubleshooting.md)

- [Concepts](ch02-concepts/00-intro.md)
    - [Two transports, one simulator](ch02-concepts/01-transports.md)
    - [The topic namespace](ch02-concepts/02-topics.md)
    - [System ids, and the two kinds of robot](ch02-concepts/03-sys-id.md)
    - [The shape of a program](ch02-concepts/04-program-shape.md)
    - [What connect actually does](ch02-concepts/05-connect.md)
    - [Five rules that explain everything](ch02-concepts/06-five-rules.md)
    - [Frames, axes and units](ch02-concepts/07-frames-and-units.md)
    - [Rotation conversions](ch02-concepts/08-rotation-conversions.md)

---

# The API in four slices

- [Reading state](ch03-reading-state/00-intro.md)
    - [Truth, measured and believed](ch03-reading-state/01-truth-measured-believed.md)
    - [Kinematics](ch03-reading-state/02-kinematics.md)
    - [Sensors](ch03-reading-state/03-sensors.md)
    - [The environment block](ch03-reading-state/04-environment.md)
    - [Actuators](ch03-reading-state/05-actuator.md)
    - [Timestamps and sequence numbers](ch03-reading-state/06-timestamps.md)
    - [Pacing your loop](ch03-reading-state/07-pacing.md)
    - [Stream health](ch03-reading-state/08-health.md)
    - [A tour of the whole snapshot](ch03-reading-state/09-sensors-tour.md)

- [Sending commands](ch04-commands/00-intro.md)
    - [Commands latch](ch04-commands/01-latching.md)
    - [Driving a multirotor](ch04-commands/02-multirotor.md)
    - [Driving the truck](ch04-commands/03-truck.md)
    - [Single degree of freedom plants](ch04-commands/04-single-dof.md)
    - [Fixed wing control](ch04-commands/05-fixed-wing.md)
    - [The generic command](ch04-commands/06-generic-cmd.md)
    - [Commands nothing acts on](ch04-commands/07-ignored-commands.md)
    - [Reading someone else's commands](ch04-commands/08-reading-commands.md)
    - [Publishing estimates](ch04-commands/09-publishing-estimates.md)
    - [The showcase attitude estimator](ch04-commands/10-showcase-estimator.md)

- [Cameras and images](ch05-cameras/00-intro.md)
    - [Mount, open and unmount](ch05-cameras/01-mount-open-unmount.md)
    - [Formats and resolution](ch05-cameras/02-formats-and-resolution.md)
    - [Inside a frame](ch05-cameras/03-frames.md)
    - [Freshness](ch05-cameras/04-freshness.md)
    - [Lens and mount pose](ch05-cameras/05-lens-and-pose.md)
    - [Two cameras at once](ch05-cameras/06-two-cameras.md)
    - [Saving a frame](ch05-cameras/07-saving-frames.md)
    - [Showing frames in a window](ch05-cameras/08-showing-frames.md)

- [Services and configuration](ch06-services/00-intro.md)
    - [Robot lifecycle](ch06-services/01-lifecycle.md)
    - [Mass and inertia](ch06-services/02-physical-params.md)
    - [Sensor noise](ch06-services/03-sensor-config.md)
    - [Coordinate frames](ch06-services/04-frames-config.md)
    - [The truck drivetrain](ch06-services/05-drive-config.md)
    - [Rotors and thrust curves](ch06-services/06-rotor-config.md)
    - [Mass spring damper and cart pole](ch06-services/07-msd-cartpole-config.md)
    - [Skins](ch06-services/08-skins.md)

---

# Reference

- [Supported virtual robots](ch07-robots/00-intro.md)
    - [Multirotor](ch07-robots/01-multirotor.md)
    - [Truck](ch07-robots/02-truck.md)
    - [Mass spring damper](ch07-robots/03-msd.md)
    - [Cart pole](ch07-robots/04-cartpole.md)
    - [Half drone](ch07-robots/05-halfdrone.md)
    - [Global Hawk](ch07-robots/06-globalhawk.md)
    - [Known simulator issues](ch07-robots/07-known-issues.md)

- [Tooling and diagnostics](ch08-tooling/00-intro.md)
    - [The vrobots command](ch08-tooling/01-cli.md)
    - [Discovery from code](ch08-tooling/02-discovery-from-code.md)
    - [Versions and pins](ch08-tooling/03-version-and-pins.md)
    - [Logging](ch08-tooling/04-logging.md)
    - [Measuring rates](ch08-tooling/05-rates.md)
    - [More than one robot](ch08-tooling/06-multi-robot.md)
    - [Recording and testing without the simulator](ch08-tooling/07-recording-and-testing.md)

---

[Appendix A: Topic reference](appendix-a-topics.md)
[Appendix B: Command reference](appendix-b-commands.md)
[Appendix C: Error reference](appendix-c-errors.md)
[Appendix D: Glossary](appendix-d-glossary.md)
