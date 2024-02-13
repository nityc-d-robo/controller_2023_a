mod p9n_interface;
mod ps5_dualsense;

// use drobo_interfaces::srv::{SolenoidStateSrv_Request, SolenoidStateSrv_Response};
use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::common_interfaces::{sensor_msgs, std_msgs::{self, msg::Bool}},
    pr_fatal, pr_info,
    selector::Selector,
    service::client::Client,
    topic::{publisher::{self, Publisher}, subscriber::Subscriber},
    RecvResult,
};


use std::{rc::Rc, time::Duration};
use drobo_interfaces::msg::MdLibMsg;

enum CANNON {
    R = 4,
    L = 5,
}

const RECOVERY:u8 = 6;
const ISOLATION:u8 = 7;

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
    let subscriber = node.create_subscriber::<sensor_msgs::msg::Joy>("joy", None)?;
    let publisher = node.create_publisher::<drobo_interfaces::msg::MdLibMsg>("md_driver_topic", None)?;



    worker(
        selector,
        subscriber,
        publisher
    )?;
    Ok(())
}

fn worker(
    mut selector: Selector,
    subscriber: Subscriber<sensor_msgs::msg::Joy>,
    publisher:Publisher<MdLibMsg>
) -> Result<(), DynError> {
    let mut p9n = p9n_interface::PlaystationInterface::new(sensor_msgs::msg::Joy::new().unwrap());
    // let logger = Rc::new(Logger::new("controller_2023"));

    selector.add_subscriber(
        subscriber,
        Box::new(move |_msg| {
            p9n.set_joy_msg(_msg.get_owned().unwrap());

            if p9n.pressed_r2() {
                cannon_controller(500,true,&publisher);
            }
            if !p9n.pressed_r2() {
                cannon_controller(0,false,&publisher);
            }


            if p9n.pressed_r1() {
                recovery_controller(999,true,&publisher);
            }
            if !p9n.pressed_r1() {
                recovery_controller(0,false,&publisher);
            }



            if p9n.pressed_dpad_down(){
                isolation_controller(500,true,&publisher);
            }
            if !p9n.pressed_dpad_down(){
                isolation_controller(0,true,&publisher);
            }



        }),
    );
    loop {
        selector.wait()?;
    }
}


fn cannon_controller(power:u16,mode:bool,publisher:&Publisher<MdLibMsg>){

    if mode {
        send_pwm(CANNON::R as u8,0,false,power,&publisher);
        send_pwm(CANNON::L as u8,0,true,power,&publisher);

        return;

    }

    send_pwm(CANNON::R as u8,0,true,0,&publisher);
    send_pwm(CANNON::L as u8,0,true,0,&publisher);
}


fn recovery_controller(power:u16,mode:bool,publisher:&Publisher<MdLibMsg>){

    if mode {
        send_pwm(RECOVERY,0,true,power,&publisher);
        return;

    }

    send_pwm(RECOVERY,0,true,0,&publisher);

}

fn isolation_controller(power:u16,mode:bool,publisher:&Publisher<MdLibMsg>){

    if mode {
        send_pwm(ISOLATION,0,true,power,&publisher);
        return;

    }

    send_pwm(ISOLATION,0,true,0,&publisher);

}


fn send_pwm(_address:u8, _semi_id:u8,_phase:bool,_power:u16,publisher:&Publisher<MdLibMsg>){
    let mut msg = drobo_interfaces::msg::MdLibMsg::new().unwrap();
    msg.address = _address as u8;
    msg.semi_id = _semi_id as u8;
    msg.mode = 2 as u8; //MotorLibのPWMモードに倣いました
    msg.phase = _phase as bool;
    msg.power = _power as u16;

    publisher.send(&msg).unwrap()

}