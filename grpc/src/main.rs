use tmotor_grpc::run_server;
use std::net::SocketAddr;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let addr: SocketAddr = "127.0.0.1:50051".parse()?;
    println!("Starting gRPC server on {}", addr);
    run_server(addr).await
}
