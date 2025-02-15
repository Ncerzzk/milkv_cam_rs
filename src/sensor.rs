use std::{io::Write, time::Duration};

use mpu6050::*;
use linux_embedded_hal::{I2cdev, Delay};
use std::time::Instant;

pub fn sensor_init(devname:&str, filename:&str){
    let i2c = I2cdev::new(devname)
    .map_err(Mpu6050Error::I2c).unwrap();

    let mut delay = Delay;
    let mut mpu = Mpu6050::new(i2c);

    mpu.init_without_verify(&mut delay).unwrap();

    println!("sensor inited!");
    let header="GYROFLOW IMU LOG
version,1.3
id,custom_logger_name
orientation,xYz
note,development_test
fwversion,FIRMWARE_0.1.0
timestamp,1644159993
vendor,ncerzzk
videofilename,videofilename.mp4
lensprofile,potatocam_mark1_prime_7_5mm_4k_xxx
tscale,0.002
gscale,1
ascale,1
t,gx,gy,gz,ax,ay,az\n";
    let mut f = std::fs::File::options().create(true).write(true).truncate(true).open(filename).unwrap();
    f.write(header.as_bytes()).unwrap();
    std::thread::spawn(move ||{

        while crate::START_RECV_FRAME.load(std::sync::atomic::Ordering::Relaxed){}

        let start_t = Instant::now();
        let mut last_t = Instant::now();
        let mut cnt = 0;

        loop{
            let t = Instant::now();
            // println!("t:{}",(t-last_t).as_micros());
            // last_t = t;
            let acc = mpu.get_acc().unwrap();
            let gyro = mpu.get_gyro().unwrap();
    
            let s: String = format!("{},{},{},{},{},{},{}\n",(t-start_t).as_millis() as u64/2,
            gyro.data.0[0][0],gyro.data.0[0][1],gyro.data.0[0][2],
             acc.data.0[0][0],acc.data.0[0][1],acc.data.0[0][2]);
            cnt += 1;
            f.write(s.as_bytes()).unwrap();
    
            let b = Instant::now() - t;
            if b.as_millis() < 2{
                std::thread::sleep( b.abs_diff(Duration::from_millis(2)));
            }


            if cnt % 500 == 0{
                f.flush().unwrap();
            }
            if crate::STOP.load(std::sync::atomic::Ordering::Relaxed) {
                f.flush().unwrap();
                break;
            }
        }
    });

}