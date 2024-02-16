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
    msg::common_interfaces::geometry_msgs::msg::Twist
};
use crate::std_msgs::msg;
use std::{rc::Rc, time::Duration};
use drobo_interfaces::msg::MdLibMsg;

enum CANNON {
    R = 4,
    L = 5,
}

const RECOVERY:u8 = 6;
const ISOLATION:u8 = 7;
const _CHARGE:u8 = 8;

struct Tire{
    id:usize,
    raito:f64
}

struct  Chassis {
    fl:Tire,
    fr:Tire,
    br:Tire, 
    bl:Tire,
}

const CHASSIS:Chassis = Chassis{
    fl:Tire{
        id:0,
        raito:1.
    },
    fr:Tire{
        id:1,
        raito:1.
    },
    br:Tire{
        id:2,
        raito:1.
    },
    bl:Tire{
        id:3,
        raito:1.
    }
};



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
    let publisher_msd = node.create_publisher::<msg::Bool>("servo_atonement_topic", None)?;
    let publisher_md = node.create_publisher::<drobo_interfaces::msg::MdLibMsg>("md_driver_topic", None)?;
    let publisher_cmd = node.create_publisher::<Twist>("md_driver_topic", None)?;



    worker(
        selector,
        subscriber,
        publisher_md,
        publisher_msd,
        publisher_cmd
        
    )?;
    Ok(())
}

fn worker(
    mut selector: Selector,
    subscriber: Subscriber<sensor_msgs::msg::Joy>,
    publisher_md:Publisher<MdLibMsg>,
    publisher_msd:Publisher<msg::Bool>,
    publisher_cmd:Publisher<Twist>
) -> Result<(), DynError> {
    let mut p9n = p9n_interface::PlaystationInterface::new(sensor_msgs::msg::Joy::new().unwrap());
    let _logger = Rc::new(Logger::new("controller_2023"));

    let mut charge_state = true;
    let mut earthquake_state = true;
    selector.add_subscriber(
        subscriber,
        Box::new(move |_msg| {
            p9n.set_joy_msg(_msg.get_owned().unwrap());

            if p9n.pressed_r2() {
                cannon_controller(400,true,&publisher_md);
            }
            if !p9n.pressed_r2() {
                cannon_controller(0,false,&publisher_md);
            }


            if p9n.pressed_r1() {

                recovery_controller(450,true,&publisher_md);
            }
            if !p9n.pressed_r1() {
                recovery_controller(0,false,&publisher_md);
            }



            if p9n.pressed_dpad_down(){
                isolation_controller(500,true,true,&publisher_md);
            }
            else if  p9n.pressed_dpad_up(){
                
                isolation_controller(500,true,false,&publisher_md);
            }
            else if !p9n.pressed_dpad_up() && !p9n.pressed_dpad_down(){
                isolation_controller(0,true,false,&publisher_md);
            }




            if p9n.pressed_dpad_left()  && charge_state{
                charge_contorller(&publisher_msd);
                charge_state = false;

            }
            if !p9n.pressed_dpad_left(){
                charge_state = true;
            }



            if p9n.pressed_dpad_right(){

                omni_control_earthquake(earthquake_state,500.,&publisher_cmd);

            }
            if !p9n.pressed_dpad_right(){
                // omni_control_earthquake(450.,&publisher_md);
            }



        }),
    );
    loop {
        selector.wait()?;
    }
}


fn cannon_controller(power:u16,mode:bool,publisher_md:&Publisher<MdLibMsg>){

    if mode {
        send_pwm(CANNON::R as u8,0,true,power,&publisher_md);
        send_pwm(CANNON::L as u8,0,false,power,&publisher_md);

        return;

    }

    send_pwm(CANNON ::R as u8,0, true,0,&publisher_md);
    send_pwm(CANNON::L as u8,0,false,0,&publisher_md);
}


fn recovery_controller(power:u16,mode:bool,publisher_md:&Publisher<MdLibMsg>){

    if mode {
        send_pwm(RECOVERY,0,true,power,&publisher_md);
        return;

    }

    send_pwm(RECOVERY,0,true,0,&publisher_md);

}


fn isolation_controller(power:u16,mode:bool,phase:bool,publisher_md:&Publisher<MdLibMsg>){

    if mode {
        send_pwm(ISOLATION,0,phase,power,&publisher_md);
        return;

    }

    send_pwm(ISOLATION,0,true,0,&publisher_md);

}


fn charge_contorller(publisher_msd:&Publisher<msg::Bool>){
    let mut msg = crate::std_msgs::msg::Bool::new().unwrap();
    msg.data = true;
    publisher_msd.send(&msg).unwrap()
}

// // これは機械の問題で本体を揺らす必要があったからです。
// fn omni_control_earthquake(_power:f64,publisher:&Publisher<MdLibMsg>){
    
//     let mut motor_power:[f64;4] = [0.;4];
//     motor_power[CHASSIS.fr.id] = -( _power as f64 * CHASSIS.fr.raito ); 
//     motor_power[CHASSIS.bl.id] =    _power as f64 * CHASSIS.bl.raito  ;
//     motor_power[CHASSIS.fl.id] = -( _power as f64 * CHASSIS.fl.raito );
//     motor_power[CHASSIS.br.id] =    _power as f64 * CHASSIS.br.raito  ;

//     for i in 0..motor_power.len() {        
//         send_pwm(i as u8,0,motor_power[i]>0.,motor_power[i] as u16,publisher);
//     }

// }

fn omni_control_earthquake(lr:bool,power:f64,publisher:&Publisher<Twist>) -> bool{
    // 
    send_twist(0., 0., {if lr {power}else {-power}}, publisher);
    !lr
}

fn send_pwm(_address:u8, _semi_id:u8,_phase:bool,_power:u16,publisher_md:&Publisher<MdLibMsg>){
    let mut msg = drobo_interfaces::msg::MdLibMsg::new().unwrap();
    msg.address = _address as u8;
    msg.semi_id = _semi_id as u8;
    msg.mode = 2 as u8; //MotorLibのPWMモードに倣いました
    msg.phase = _phase as bool;
    msg.power = _power as u16;

    publisher_md.send(&msg).unwrap()

}

fn send_twist(_x:f64,_y:f64,_z:f64, publisher:&Publisher<Twist>){
    let mut msg = Twist::new().unwrap();

    msg.angular.z = _z;
    msg.linear.x  = _x;
    msg.linear.y  = _y;


}