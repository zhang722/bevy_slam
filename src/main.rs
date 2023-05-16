use std::io::{self, Write};
use std::net::TcpStream;
use std::thread;
use std::time::Duration;

mod slam;

fn main() -> io::Result<()> {
    // 连接到目标 TCP 服务器
    let mut stream = TcpStream::connect("127.0.0.1:9123")?;
    let mut i = 0;

    // 循环发送消息
    loop {
        let message1 = format!("{{body {} 0.0 0.0 0.0 1.0 0.01 0.01 0.01}}", i);
        let message2 = format!("{{point {} {} 0.0 0.0}}", i, 0.1 * i as f32);
        i += 1;
        // let message = "Hello, TCP server!";
        stream.write_all(message1.as_bytes())?;
        stream.write_all(message2.as_bytes())?;

        // 暂停一段时间
        thread::sleep(Duration::from_micros(10000));
    }
}