use tonic::{transport::Server, Request, Response, Status};
use tokio::sync::mpsc;
use tokio_stream::wrappers::ReceiverStream;

pub mod motor {
    tonic::include_proto!("tmotor");
}

use motor::{motor_service_server::{MotorService, MotorServiceServer}, MotorCommand, MotorState};

#[derive(Default)]
pub struct MotorServer;

#[tonic::async_trait]
impl MotorService for MotorServer {
    async fn send_command(
        &self,
        _request: Request<MotorCommand>,
    ) -> Result<Response<MotorState>, Status> {
        let reply = MotorState { position: 0.0, velocity: 0.0, torque: 0.0 };
        Ok(Response::new(reply))
    }

    type StreamStateStream = ReceiverStream<Result<MotorState, Status>>;

    async fn stream_state(
        &self,
        _request: Request<MotorCommand>,
    ) -> Result<Response<Self::StreamStateStream>, Status> {
        let (tx, rx) = mpsc::channel(4);
        let _ = tx.send(Ok(MotorState { position: 0.0, velocity: 0.0, torque: 0.0 })).await;
        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

pub async fn run_server(addr: std::net::SocketAddr) -> Result<(), Box<dyn std::error::Error>> {
    let motor_server = MotorServer::default();
    Server::builder()
        .add_service(MotorServiceServer::new(motor_server))
        .serve(addr)
        .await?;
    Ok(())
}
