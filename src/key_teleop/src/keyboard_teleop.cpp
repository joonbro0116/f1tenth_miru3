#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <algorithm>
#include <string>

using ackermann_msgs::msg::AckermannDriveStamped;

namespace {
struct TermiosGuard {
  termios orig{}; bool active{false};
  void enableRaw() {
    if (tcgetattr(STDIN_FILENO, &orig) == 0) {
      termios raw = orig; raw.c_lflag &= ~(ICANON | ECHO);
      raw.c_cc[VMIN] = 0; raw.c_cc[VTIME] = 0;
      tcsetattr(STDIN_FILENO, TCSANOW, &raw); active = true;
    }
  }
  ~TermiosGuard(){ if(active) tcsetattr(STDIN_FILENO, TCSANOW, &orig); }
};
bool kbhit(){ timeval tv{0,0}; fd_set fds; FD_ZERO(&fds); FD_SET(STDIN_FILENO,&fds);
  return select(STDIN_FILENO+1,&fds,nullptr,nullptr,&tv)>0; }
int getch(){ unsigned char c; return read(STDIN_FILENO,&c,1)==1?c:-1; }
template<typename T> T clamp(T v,T lo,T hi){ return std::max(lo,std::min(hi,v)); }
}

class KeyboardTeleop : public rclcpp::Node {
public:
  KeyboardTeleop():Node("key_teleop"){
    declare_parameter<std::string>("ego_namespace","ego_racecar");
    declare_parameter<std::string>("drive_topic","drive");
    declare_parameter<double>("publish_hz",30.0);
    declare_parameter<double>("max_speed",4.0);
    declare_parameter<double>("max_steer",0.36);
    declare_parameter<double>("accel_step",0.2);
    declare_parameter<double>("steer_step",0.04);
    declare_parameter<bool>("invert_steer",false);
    get_params_();

    std::string full_topic = "/" + (ns_.empty()?std::string():ns_+"/") + drive_topic_;
    pub_ = create_publisher<AckermannDriveStamped>(full_topic,10);
    RCLCPP_INFO(get_logger(),"Publishing to: %s", full_topic.c_str());

    term_.enableRaw(); print_help_();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0/publish_hz_),
                               std::bind(&KeyboardTeleop::tick_, this));
  }
private:
  void get_params_(){
    get_parameter("ego_namespace",ns_);
    get_parameter("drive_topic",drive_topic_);
    get_parameter("publish_hz",publish_hz_);
    get_parameter("max_speed",max_speed_);
    get_parameter("max_steer",max_steer_);
    get_parameter("accel_step",accel_step_);
    get_parameter("steer_step",steer_step_);
    get_parameter("invert_steer",invert_);
  }
  void tick_(){
    if(kbhit()){ int c=getch(); if(c>=0) handle_key_((char)c); }
    AckermannDriveStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    msg.drive.speed = v_;
    msg.drive.steering_angle = delta_;
    pub_->publish(msg);
  }
  void handle_key_(char c){
    switch(c){
      case 'w': case 'W': v_ = clamp(v_ + accel_step_, -max_speed_, max_speed_); break;
      case 's': case 'S': v_ = clamp(v_ - accel_step_, -max_speed_, max_speed_); break;
      case ' ': v_ = 0.0; break;
      case 'a': case 'A': delta_ = clamp(delta_ + (invert_?-steer_step_:+steer_step_), -max_steer_, +max_steer_); break;
      case 'd': case 'D': delta_ = clamp(delta_ + (invert_?+steer_step_:-steer_step_), -max_steer_, +max_steer_); break;
      case 'x': case 'X': delta_ = 0.0; break;
      case 'q': case 'Q': RCLCPP_INFO(get_logger(),"Quit."); rclcpp::shutdown(); return;
      case 'h': case 'H': print_help_(); break;
      default: break;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200, "speed=%.2f, steer=%.3f", v_, delta_);
  }
  void print_help_(){
    printf("\n=== Key Teleop ===\n"
           "W/S: speed +/- (space: stop)\n"
           "A/D: steer +/- (X: center)\n"
           "H: help  Q: quit\n"
           "max_speed=%.2f, max_steer=%.3f, invert=%s\n\n",
           max_speed_, max_steer_, invert_?"true":"false");
    fflush(stdout);
  }
  TermiosGuard term_;
  rclcpp::Publisher<AckermannDriveStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string ns_, drive_topic_;
  double publish_hz_{30.0}, max_speed_{4.0}, max_steer_{0.36}, accel_step_{0.2}, steer_step_{0.04};
  bool invert_{false};
  double v_{0.0}, delta_{0.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleop>());
  rclcpp::shutdown();
  return 0;
}
