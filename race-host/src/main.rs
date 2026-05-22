use std::io::{Read, Write};
use std::time::Duration;

use race_core::gnss::{GnssReceiver, GnssUpdate, Um980Receiver};
use race_core::run::{RunConfig, RunDetector, RunEvent};

fn main() {
    let mut receiver = Um980Receiver::new();
    let baud = receiver.baudrate();

    let port_name = serialport::available_ports()
        .unwrap()
        .into_iter()
        .next()
        .expect("no serial ports found")
        .port_name;
    println!("Using {port_name} @ {baud}");

    let mut port = serialport::new(&port_name, baud)
        .timeout(Duration::from_millis(500))
        .open()
        .unwrap();

    // port.write_all(&receiver.init_bytes()).unwrap();
    port.flush().unwrap();

    let mut detector = RunDetector::new(RunConfig::default());
    let mut chunk = [0u8; 256];

    loop {
        let Ok(n) = port.read(&mut chunk) else {
            continue;
        };

        receiver.consume(&chunk[..n], &mut |update| match update {
            GnssUpdate::Fix(fix) => {
                let event = detector.update(&fix);
                if fix.has_fix {
                    println!(
                        "Latitude: {:.5} Longitude: {:.5} Altitude: {:.2}m",
                        fix.lat, fix.lon, fix.alt_m
                    );
                    println!(
                        "Speed: {:.2} km/h Accel: {:.2} m/s^2 Heading: {:.2} degrees",
                        fix.speed_mps * 3.6,
                        detector.accel_mps2,
                        fix.heading_deg
                    );
                }
                match event {
                    Some(RunEvent::Armed) => println!("Armed — waiting for launch"),
                    Some(RunEvent::Launched) => println!("Launch! timing..."),
                    Some(RunEvent::Finished { time_s, distance_m }) => {
                        println!("0-target in {time_s:.3} s over {distance_m:.1} m")
                    }
                    Some(RunEvent::Aborted) => println!("Run aborted"),
                    None => {}
                }
            }
            GnssUpdate::Sats { count, best_cno } => {
                println!("Sats: {count} visible, best C/N0 {best_cno} dBHz");
            }
        });
    }
}
