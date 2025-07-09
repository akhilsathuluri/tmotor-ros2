# Using the gRPC Interface

This guide explains how to build and run the Rust based gRPC service and how it corresponds to the original ROS2 Python nodes.

## Building

The service lives in the `grpc/` directory. Build it with Cargo:

```bash
cd grpc
cargo build --release
```

This produces a binary named `tmotor_grpc_server` in `target/release/`.

## Running the server

From the repository root run:

```bash
cargo run --package tmotor_grpc --bin tmotor_grpc_server
```

The server listens on `127.0.0.1:50051` by default. Clients can connect to this address using gRPC to send motor commands and stream motor state.

## Mapping from ROS2 to gRPC

The previous Python implementation used ROS2 topics defined in `motor_testbench` and messages in `motor_testbench_msgs`.

| ROS2 Component | gRPC Equivalent |
| -------------- | --------------- |
| `TmotorCmd` message | `MotorCommand` proto message |
| `TmotorData` message | `MotorState` proto message |
| Publishers on `motor_command/motor_<id>` | `SendCommand` RPC call |
| Subscribers on `can_response/motor_<id>` | `StreamState` streaming RPC |

`motor_message_passer.py` and `motor_commander.py` publish and subscribe to these topics. When migrating, their logic should be moved into a gRPC client that calls `SendCommand` and consumes the `StreamState` stream.

## Example client snippet

```rust
use tmotor_grpc::motor::motor_service_client::MotorServiceClient;
use tmotor_grpc::motor::MotorCommand;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut client = MotorServiceClient::connect("http://127.0.0.1:50051").await?;
    let request = tonic::Request::new(MotorCommand{ status: true, setzero: false, position: 0.0, velocity: 0.0, torque: 0.0, kp: 0.0, kd: 0.0});
    let response = client.send_command(request).await?;
    println!("Received state: {:?}", response.into_inner());
    Ok(())
}
```

This demonstrates sending a single command and printing the returned state. More advanced clients can use `stream_state` to receive continuous updates.
