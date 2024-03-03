use safe_drive::{
    context::Context, 
    error::DynError, 
    logger::Logger, 
    msg::{common_interfaces::{nav_msgs, sensor_msgs}, RosString}, 
    pr_info, 
    topic::{publisher::Publisher, subscriber::Subscriber}
};

use async_std;
use ekf_utils;

#[async_std::main]
async fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;
    let node = ctx.create_node("posture_estimater", None, Default::default())?;

    let subscriber = node.create_subscriber::<sensor_msgs::msg::Imu>("/imu", None)?;

    let publisher = node.create_publisher::<nav_msgs::msg::Odometry>("/odom", None)?;

    let task = async_std::task::spawn(kalman_filter_task(subscriber,publisher, 10));

    task.await?;

    Ok(())
}

async fn kalman_filter_task(
    mut sub_imu:Subscriber<sensor_msgs::msg::Imu>,
    pub_odom:Publisher<nav_msgs::msg::Odometry>,
    delta_milli:u64
)->Result<(), DynError>
{
    let log = Logger::new("imu_posture");

    let mut odom = nav_msgs::msg::Odometry::new().unwrap();
    odom.header.frame_id = RosString::new("odom").unwrap();

    let mut ekf = ekf_utils::posture::GyroAccelEKF::new(delta_milli as f32 * 10e-4);

    pr_info!(log, "Start IMUPosture on deltatime:/{}s", delta_milli as f32 * 10e-4);

    loop {
        let get_imu = sub_imu.recv().await.unwrap();
        let gyro = ekf_utils::convert_to_vector(
            get_imu.angular_velocity.x.to_radians() as f32, 
            get_imu.angular_velocity.y.to_radians() as f32, 
            get_imu.angular_velocity.z.to_radians() as f32);

        let accel = ekf_utils::convert_to_vector(
            get_imu.linear_acceleration.x as f32, 
            get_imu.linear_acceleration.y as f32, 
            get_imu.linear_acceleration.z as f32);

        let result = ekf.run_ekf(gyro, accel, delta_milli as f32 * 10e-4);
        let q = ekf_utils::xyz_to_quaternion(result);

        odom.pose.pose.orientation.w = q.w as f64;
        odom.pose.pose.orientation.x = q.i as f64;
        odom.pose.pose.orientation.y = q.j as f64;
        odom.pose.pose.orientation.z = q.k as f64;

        let _ = pub_odom.send(&odom);
        std::thread::sleep(std::time::Duration::from_millis(delta_milli));
    }
}