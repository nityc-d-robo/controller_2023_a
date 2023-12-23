mod p9n_interface;
mod ps5_dualsense;

// use drobo_interfaces::srv::{SolenoidStateSrv_Request, SolenoidStateSrv_Response};
use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::common_interfaces::{sensor_msgs, std_msgs},
    pr_fatal, pr_info,
    selector::Selector,
    service::client::Client,
    topic::{publisher::Publisher, subscriber::Subscriber},
    RecvResult,
};
use std::{rc::Rc, time::Duration};

pub mod DualsenseState {
    pub const SQUARE: usize = 0;
    pub const CIRCLE: usize = 1;
    pub const TRIANGLE: usize = 2;
    pub const CROSS: usize = 3;
    pub const L1: usize = 4;
    pub const L2: usize = 5;
    pub const R1: usize = 6;
    pub const R2: usize = 7;
    pub const D_PAD_UP: usize = 8;
    pub const D_PAD_DOWN: usize = 9;
    pub const D_PAD_LEFT: usize = 10;
    pub const D_PAD_RIGHT: usize = 11;
    pub const START: usize = 12;
    pub const SELECT: usize = 13;
    pub const PS: usize = 14;
}

fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;
    let node = ctx.create_node("controller_2023_a", None, Default::default())?;

    let selector = ctx.create_selector()?;
    let selector_client = ctx.create_selector()?;
    let subscriber = node.create_subscriber::<sensor_msgs::msg::Joy>("joy", None)?;

    let demeter_publisher = node.create_publisher::<std_msgs::msg::Int8>("demeter_oracle", None)?;
    let sr_publisher = node.create_publisher::<std_msgs::msg::Bool>("sr_driver_topic", None)?;
    let support_wheel_publisher =
        node.create_publisher::<std_msgs::msg::Int32>("support_drive_topic", None)?;
    // let client = node.create_client::<drobo_interfaces::srv::SolenoidStateSrv>(
    //     "solenoid_order",
    //     Default::default(),
    // )?;

    worker(
        selector,
        selector_client,
        subscriber,
        demeter_publisher,
        support_wheel_publisher,
        sr_publisher
        // client,
    )?;
    Ok(())
}

fn worker(
    mut selector: Selector,
    mut selector_client: Selector,
    subscriber: Subscriber<sensor_msgs::msg::Joy>,
    demeter_publisher: Publisher<std_msgs::msg::Int8>,
    support_wheel_publisher: Publisher<std_msgs::msg::Int32>,
    sr_publisher: Publisher<std_msgs::msg::Bool>
    // client: Client<drobo_interfaces::srv::SolenoidStateSrv>,
) -> Result<(), DynError> {
    let mut p9n = p9n_interface::PlaystationInterface::new(sensor_msgs::msg::Joy::new().unwrap());
    // let mut client = Some(client);
    let logger = Rc::new(Logger::new("controller_2023"));
    let logger2 = logger.clone();
    let mut dualsense_state: [bool; 15] = [false; 15];
    let mut sr_state = true;
    let mut support_wheel_prioritize = 0; // 1: 前, -1: 後ろ
    selector.add_subscriber(
        subscriber,
        Box::new(move |_msg| {
            p9n.set_joy_msg(_msg.get_owned().unwrap());

            if p9n.pressed_start() && !dualsense_state[DualsenseState::START] {
                dualsense_state[DualsenseState::START] = true;
                let mut msg = std_msgs::msg::Bool::new().unwrap();
                msg.data = !sr_state;
                sr_publisher.send(&msg).unwrap();
                sr_state ^= true;
            }
            if !p9n.pressed_start() && dualsense_state[DualsenseState::START] {
                dualsense_state[DualsenseState::START] = false;
            }
            //  && !dualsense_state[DualsenseState::L1] 
            if p9n.pressed_l1(){
                pr_info!(logger, "L1");
            }
            
            // && dualsense_state[DualsenseState::L1]
            if !p9n.pressed_l1()  {
                pr_info!(logger, "L1f");
            }
            // && !dualsense_state[DualsenseState::R1] 
            if p9n.pressed_r1() {
                pr_info!(logger, "r1");
            }
            // && dualsense_state[DualsenseState::R1] 
            if !p9n.pressed_r1() {
                pr_info!(logger, "r1f");
            }
            
            /*
            if p9n.pressed_l2() {
                pr_info!(logger, "収穫機構: 上昇！");
            }
            if !p9n.pressed_l2() && dualsense_state[DualsenseState::L2] {
                pr_info!(logger, "収穫機構: 上昇！");
            }
            if p9n.pressed_r2() {
                pr_info!(logger, "収穫機構: 上昇！");
            }
            if !p9n.pressed_r2() && dualsense_state[DualsenseState::R2] {
                pr_info!(logger, "収穫機構: 上昇！");
            }

            if p9n.pressed_dpad_left() && !dualsense_state[DualsenseState::D_PAD_LEFT] {
                pr_info!(logger, "収穫機構: 上昇！");
            }
            if !p9n.pressed_dpad_left() && dualsense_state[DualsenseState::D_PAD_LEFT] {
                pr_info!(logger, "収穫機構: 上昇！");
            }

            if p9n.pressed_dpad_up() && !dualsense_state[DualsenseState::D_PAD_UP] {
                pr_info!(logger, "収穫機構: 上昇！");
            }
            if !p9n.pressed_dpad_up() && dualsense_state[DualsenseState::D_PAD_UP] {
                pr_info!(logger, "収穫機構: 上昇！");
            }

            if p9n.pressed_dpad_right() && !dualsense_state[DualsenseState::D_PAD_RIGHT] {
                pr_info!(logger, "収穫機構: 上昇！");
            }
            if !p9n.pressed_dpad_right() && dualsense_state[DualsenseState::D_PAD_RIGHT] {
                pr_info!(logger, "収穫機構: 上昇！");
            }
            */
        }),
    );
    loop {
        selector.wait()?;
    }
}