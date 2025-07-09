# TMotor gRPC Interface

This repository originally relied on ROS2 for communication with the motor
testbench.  A new Rust crate located in `grpc/` demonstrates how the same
messages can be exchanged over gRPC.

## Building the gRPC Server

```bash
cd grpc
cargo build
```

`protoc` must be available for the build to succeed. On Ubuntu install it via:

```bash
sudo apt-get install protobuf-compiler
```

The server uses [`tonic`](https://github.com/hyperium/tonic) and listens on
`127.0.0.1:50051` by default.

## Running Tests

```
cd grpc
cargo test
```

Tests are minimal at the moment and simply exercise the Rust tool chain.

For a detailed migration strategy see `docs/grpc_migration_plan.md`. Usage
instructions and a mapping between the legacy ROS2 nodes and the new gRPC
components are available in `docs/grpc_usage.md`.
