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
        let message = format!("Hello, TCP server{}!\n", i);
        i += 1;
        // let message = "Hello, TCP server!";
        stream.write_all(message.as_bytes())?;

        // 暂停一段时间
        thread::sleep(Duration::from_secs(1));
    }
}