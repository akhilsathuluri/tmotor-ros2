# gRPC Migration Plan

This document outlines the high level steps required to replace the existing ROS2
communication layer with a gRPC based interface written in Rust. The goal is to
preserve the existing functionality while enabling communication with Drake via a
digital twin interface.

## 1. Analyse Existing ROS2 Usage
- Identify all ROS2 nodes and topics in `motor_testbench` and `node_launcher`.
- Determine the message types published and subscribed (`TmotorCmd`, `TmotorData`).
- Map these messages to gRPC service calls.

## 2. Define gRPC Interfaces
- Create a `proto` file describing motor command and state messages.
- Implement a `MotorService` with RPCs to send commands and stream motor state.
- Consider Flatbuffers for serialization if performance is critical.

## 3. Implement Rust gRPC Server
- Create a new Rust crate under `grpc/` using [`tonic`](https://github.com/hyperium/tonic).
- The server will expose the `MotorService` and forward received commands to the
  motor driver or simulator.
- Provide a client API for other components (e.g. Drake) to interact with the
  motors.

## 4. Replace Python Nodes
- Gradually port existing Python nodes to Rust binaries that use the gRPC client.
- Initially maintain a thin compatibility layer that translates between ROS2 and
  gRPC so existing tooling can still be used during migration.

## 5. Integrate with Drake
- Use gRPC to send joint commands from Drake and receive state updates to build a
  digital twin connection.
- Provide examples on how to launch the server alongside a Drake simulation.

## 6. Testing
- Add unit tests for the Rust crate using `cargo test`.
- Provide integration tests (Python or Rust) that start a test server and verify
  command/state round trips.

## 7. Documentation
- Extend the project README with build instructions for the Rust components.
- Document the gRPC API and how to run the server with the simulator.

This plan aims to incrementally transition the project away from ROS2 while
keeping functionality intact during the migration.
