use std::{io::Write, time::Duration};

use mpu6050::*;
use linux_embedded_hal::{I2cdev, Delay};
use i2cdev::linux::LinuxI2CError;
use std::time::Instant;

pub fn sensor_init(){
    let i2c = I2cdev::new("/dev/i2c-1")
    .map_err(Mpu6050Error::I2c).unwrap();

    let mut delay = Delay;
    let mut mpu = Mpu6050::new(i2c);

    mpu.init(&mut delay).unwrap();

    println!("sensor inited!");
    let header="GYROFLOW IMU LOG
version,1.3
id,custom_logger_name
orientation,YxZ
note,development_test
fwversion,FIRMWARE_0.1.0
timestamp,1644159993
vendor,ncerzzk
videofilename,videofilename.mp4
lensprofile,potatocam_mark1_prime_7_5mm_4k_xxx
tscale,0.001
gscale,1
ascale,1
t,gx,gy,gz,ax,ay,az\n";
    let mut f = std::fs::File::options().create(true).write(true).open("milkv_out.gcsv").unwrap();
    f.write(header.as_bytes());
    std::thread::spawn(move ||{

        while crate::START_RECV_FRAME.load(std::sync::atomic::Ordering::SeqCst){}

        let mut t = Instant::now();
        let mut cnt = 0;
        loop{
            let acc = mpu.get_acc().unwrap();
            let gyro = mpu.get_gyro().unwrap();
    
            let s = format!("{},{},{},{},{},{},{}\n",cnt,
            gyro.data.0[0][0],gyro.data.0[0][1],gyro.data.0[0][2],
            acc.data.0[0][0],acc.data.0[0][1],acc.data.0[0][2]);
            cnt += 1;
            f.write(s.as_bytes());
    
            let b = Instant::now() - t;
            t = Instant::now();
            if b < Duration::from_millis(1) {
                std::thread::sleep( b.abs_diff(Duration::from_millis(1)));
            }

            if crate::STOP.load(std::sync::atomic::Ordering::SeqCst){
                f.flush();
                break;
            }
        }
    });

}