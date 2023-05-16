use std::io::{Read};
use std::net::{TcpListener, TcpStream};

use bevy::prelude::*;
use crossbeam_channel::{bounded, Receiver, Sender};

#[derive(Resource)]
struct Server {
    listener: TcpListener,
}

#[derive(Resource, Deref)]
struct StreamReceiver(Receiver<String>);
pub struct StreamEvent(pub String);

pub struct ServerPlugin;

impl Plugin for ServerPlugin {
    fn build(&self, app: &mut App) {
        let address = "127.0.0.1:9123";

        let listener = TcpListener::bind(address).expect("Failed to bind");
        println!("Server listening on {}", address);

        app.add_event::<StreamEvent>()
            .insert_resource(Server { listener })
            .add_startup_system(server_system)
            .add_system(read_stream);
    }
}

fn handle_client(stream: &mut TcpStream, tx: &Sender<String>)  {
    let mut buffer = [0; 1024];
    loop {
        match stream.read(&mut buffer) {
            Ok(0) => {
                // Connection closed by the client
                break;
            }
            Ok(n) => {
                // Process the received message
                let message = String::from_utf8_lossy(&buffer[..n]).to_string();
                tx.send(message).unwrap();
            }
            Err(err) => {
                eprintln!("Error reading from socket: {}", err);
                break;
            }
        }
    }
}

fn server_system(
    mut commands: Commands, 
    server: Res<Server>,
) {
    let (tx, rx) = bounded::<String>(10);
    let incoming = server.listener.try_clone().expect("");
    std::thread::spawn(move || {
        for stream in incoming.incoming() {
            match stream {
                Ok(mut stream) => {
                    // Spawn a new thread to handle the client connection
                    handle_client(&mut stream, &tx);
                }
                Err(err) => {
                    eprintln!("Error accepting connection: {}", err);
                }
            }
        }
    });
    commands.insert_resource(StreamReceiver(rx));
}


// This system reads from the receiver and sends events to Bevy
fn read_stream(
    receiver: Res<StreamReceiver>, 
    mut events: EventWriter<StreamEvent>
) {
    for from_stream in receiver.try_iter() {
        events.send(StreamEvent(from_stream));
    }
}
