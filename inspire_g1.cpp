#include "inspire.h"
#include "param.h"

#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/common/thread/recurrent_thread.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <glob.h>
#include <csignal>
#include <atomic>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <cctype>

class RawTerminal
{
public:
  RawTerminal()
  {
    if (isatty(STDIN_FILENO))
    {
      if (tcgetattr(STDIN_FILENO, &orig_termios_) == 0)
      {
        termios raw = orig_termios_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0)
        {
          active_ = true;
        }
      }
    }
  }

  ~RawTerminal()
  {
    restore();
  }

  void restore()
  {
    if (active_)
    {
      tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
      active_ = false;
    }
  }

  int readKey()
  {
    unsigned char c;
    if (active_)
    {
      int n = ::read(STDIN_FILENO, &c, 1);
      if (n > 0)
      {
        return c;
      }
      return -1;
    }
    else
    {
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(STDIN_FILENO, &rfds);
      struct timeval tv = {0, 0};
      int ret = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
      if (ret > 0)
      {
        int n = ::read(STDIN_FILENO, &c, 1);
        if (n > 0)
        {
          return c;
        }
      }
      return -1;
    }
  }

  bool isActive() const { return active_; }

private:
  bool active_ = false;
  termios orig_termios_;
};

static RawTerminal* g_raw_term = nullptr;
static std::atomic<bool> g_interrupted{false};

static void sigint_handler(int)
{
  g_interrupted = true;
  if (g_raw_term)
  {
    g_raw_term->restore();
  }
  std::cout << "\nProgram interrupted by user (Ctrl+C).\n";
  exit(0);
}

class InspireRunner
{
public:
  InspireRunner(const std::string& right_port, int right_id,
                const std::string& left_port, int left_id)
  {
    if (!right_port.empty())
    {
      std::cout << " -> Configuring Right Hand on: " << right_port
                << " (Slave ID: " << right_id << ")" << std::endl;
      serial1 = std::make_shared<SerialPort>(right_port, B115200, 15);
      if (serial1->is_open())
      {
        righthand = std::make_shared<inspire::InspireHand>(serial1, right_id > 0 ? right_id : 1);
        righthand->ClearError();
      }
      else
      {
        std::cerr << " -> Error: Could not open Right Hand port: " << right_port << std::endl;
      }
    }

    if (!left_port.empty())
    {
      std::cout << " -> Configuring Left Hand on:  " << left_port
                << " (Slave ID: " << left_id << ")" << std::endl;
      serial2 = std::make_shared<SerialPort>(left_port, B115200, 15);
      if (serial2->is_open())
      {
        lefthand = std::make_shared<inspire::InspireHand>(serial2, left_id > 0 ? left_id : 1);
        lefthand->ClearError();
      }
      else
      {
        std::cerr << " -> Error: Could not open Left Hand port: " << left_port << std::endl;
      }
    }

    qcmd.setZero();
    qstate.setZero();
    fstate.setZero();

    // dds setup
    handcmd = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>>(
        "rt/" + param::ns + "/cmd");
    handcmd->msg_.cmds().resize(12);
    handstate = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>>(
        "rt/" + param::ns + "/state");
    handstate->msg_.states().resize(12);

    // Start recurrent thread at 30ms (33Hz)
    thread = std::make_shared<unitree::common::RecurrentThread>(
      30000, std::bind(&InspireRunner::run, this)
    );
  }

  void run()
  {
    // Set command
    if(!handcmd->isTimeout())
    {
      for(int i = 0; i < 12; i++)
      {
        qcmd(i) = handcmd->msg_.cmds()[i].q();
      }
      if (righthand)
      {
        righthand->SetPosition(qcmd.block<6, 1>(0, 0));
      }
      if (lefthand)
      {
        lefthand->SetPosition(qcmd.block<6, 1>(6, 0));
      }
    }

    // Recv state
    Eigen::Matrix<double, 6, 1> qtemp, ftemp;
    static int force_poll_counter = 0;
    bool poll_force_this_tick = ((force_poll_counter++ % 4) == 0);

    if (righthand)
    {
      int status = righthand->GetPosition(qtemp);
      if(status == 0)
      {
        qstate.block<6, 1>(0, 0) = qtemp;
      }
      else
      {
        for(int i = 0; i < 6; i++)
        {
          handstate->msg_.states()[i].lost()++;
        }
      }

      if (poll_force_this_tick)
      {
        int fstatus = righthand->GetForce(ftemp);
        if(fstatus == 0)
        {
          fstate.block<6, 1>(0, 0) = ftemp;
        }
      }
    }

    if (lefthand)
    {
      int status = lefthand->GetPosition(qtemp);
      if(status == 0)
      {
        qstate.block<6, 1>(6, 0) = qtemp;
      }
      else
      {
        for(int i = 0; i < 6; i++)
        {
          handstate->msg_.states()[i+6].lost()++;
        }
      }

      if (poll_force_this_tick)
      {
        int fstatus = lefthand->GetForce(ftemp);
        if(fstatus == 0)
        {
          fstate.block<6, 1>(6, 0) = ftemp;
        }
      }
    }

    if(handstate->trylock())
    {
      for(int i = 0; i < 12; i++)
      {
        handstate->msg_.states()[i].q() = qstate(i);
        handstate->msg_.states()[i].tau_est() = fstate(i);
      }
      handstate->unlockAndPublish();
    }
  }

  unitree::common::ThreadPtr thread;

  SerialPort::SharedPtr serial1;
  SerialPort::SharedPtr serial2;
  std::shared_ptr<inspire::InspireHand> lefthand;
  std::shared_ptr<inspire::InspireHand> righthand;
  Eigen::Matrix<double, 12, 1> qcmd, qstate, fstate;

  std::unique_ptr<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>> handstate;
  std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>> handcmd;
};

static std::vector<std::string> getAvailablePorts()
{
  std::vector<std::string> ports;
  glob_t glob_result;
  memset(&glob_result, 0, sizeof(glob_result));
  if (glob("/dev/ttyUSB*", 0, NULL, &glob_result) == 0)
  {
    for (size_t i = 0; i < glob_result.gl_pathc; ++i)
    {
      ports.push_back(std::string(glob_result.gl_pathv[i]));
    }
  }
  globfree(&glob_result);
  std::sort(ports.begin(), ports.end());

  for (int i = 0; i <= 3; ++i)
  {
    std::string default_port = "/dev/ttyUSB" + std::to_string(i);
    if (std::find(ports.begin(), ports.end(), default_port) == ports.end())
    {
      ports.push_back(default_port);
    }
  }
  return ports;
}

static bool testAndSelectHand(const std::string& hand_name,
                              std::vector<std::string>& candidate_ports,
                              RawTerminal& term,
                              std::string& selected_port,
                              int& selected_id)
{
  if (candidate_ports.empty())
  {
    std::cout << "No candidate ports available to test for " << hand_name << " hand.\n";
    return false;
  }

  size_t port_idx = 0;

  while (!g_interrupted)
  {
    std::string current_port = candidate_ports[port_idx];

    std::cout << "============================================================\n";
    std::cout << " [Testing " << hand_name << " HAND] -> " << current_port << "\n";
    std::cout << " Cycling fingers OPEN <-> CLOSE (Single-keystroke controls):\n";
    std::cout << "   [y] : Confirm " << hand_name << " hand is moving on " << current_port << "\n";
    std::cout << "   [n] : Not working / wrong hand (Try next port)\n";
    std::cout << "   [s] : Skip " << hand_name << " hand\n";
    std::cout << "   [q] : Quit\n";
    std::cout << "============================================================\n" << std::flush;

    auto serial = std::make_shared<SerialPort>(current_port, B115200, 15);
    if (!serial->is_open())
    {
      std::cout << " -> Warning: Cannot open " << current_port << ".\n";
      std::cout << "    Press [n] to try next port, [s] to skip, [q] to quit: " << std::flush;

      while (!g_interrupted)
      {
        int k = term.readKey();
        if (k == 'n' || k == 'N' || k == ' ')
        {
          port_idx = (port_idx + 1) % candidate_ports.size();
          std::cout << "\n\n";
          break;
        }
        else if (k == 's' || k == 'S')
        {
          std::cout << "\n -> " << hand_name << " hand SKIPPED.\n\n";
          selected_port = "";
          selected_id = -1;
          return false;
        }
        else if (k == 'q' || k == 'Q')
        {
          term.restore();
          std::cout << "\nSetup cancelled by user.\n";
          exit(0);
        }
        usleep(50000);
      }
      continue;
    }

    // Auto-detect slave ID 1 or 2
    Eigen::Matrix<double, 6, 1> dummy_q;
    int detected_id = 0;
    auto test_id1 = std::make_shared<inspire::InspireHand>(serial, 1);
    if (test_id1->GetPosition(dummy_q) == 0)
    {
      detected_id = 1;
    }
    else
    {
      auto test_id2 = std::make_shared<inspire::InspireHand>(serial, 2);
      if (test_id2->GetPosition(dummy_q) == 0)
      {
        detected_id = 2;
      }
    }

    std::shared_ptr<inspire::InspireHand> hand_primary;
    std::shared_ptr<inspire::InspireHand> hand_secondary;

    if (detected_id == 1)
    {
      hand_primary = test_id1;
    }
    else if (detected_id == 2)
    {
      hand_primary = std::make_shared<inspire::InspireHand>(serial, 2);
    }
    else
    {
      // Neither responded to read query yet; send movement to both ID 1 and ID 2
      hand_primary = test_id1;
      hand_secondary = std::make_shared<inspire::InspireHand>(serial, 2);
    }

    if (hand_primary) hand_primary->ClearError();
    if (hand_secondary) hand_secondary->ClearError();

    Eigen::Matrix<double, 6, 1> q_open;
    q_open.setOnes();
    Eigen::Matrix<double, 6, 1> q_close;
    q_close.setZero();

    bool is_open = false;
    int tick = 0;
    bool advance_port = false;

    while (!g_interrupted)
    {
      // Toggle open/close every 12 ticks (~600ms)
      if (tick % 12 == 0)
      {
        is_open = !is_open;
        const auto& q_target = is_open ? q_open : q_close;
        hand_primary->SetPosition(q_target);
        if (hand_secondary)
        {
          hand_secondary->SetPosition(q_target);
        }
      }

      int k = term.readKey();
      if (k == 'y' || k == 'Y' || k == '\r' || k == '\n')
      {
        // Confirmed
        hand_primary->SetPosition(q_open);
        if (hand_secondary) hand_secondary->SetPosition(q_open);
        serial->close_port();

        selected_port = current_port;
        selected_id = (detected_id > 0) ? detected_id : 1;
        std::cout << "\n -> SUCCESS: " << hand_name << " hand confirmed on "
                  << selected_port << " (Slave ID: " << selected_id << ")!\n\n";
        return true;
      }
      else if (k == 'n' || k == 'N' || k == ' ')
      {
        hand_primary->SetPosition(q_open);
        if (hand_secondary) hand_secondary->SetPosition(q_open);
        serial->close_port();
        port_idx = (port_idx + 1) % candidate_ports.size();
        std::cout << "\n -> Trying next port...\n\n";
        advance_port = true;
        break;
      }
      else if (k == 's' || k == 'S')
      {
        hand_primary->SetPosition(q_open);
        if (hand_secondary) hand_secondary->SetPosition(q_open);
        serial->close_port();
        std::cout << "\n -> " << hand_name << " hand SKIPPED.\n\n";
        selected_port = "";
        selected_id = -1;
        return false;
      }
      else if (k == 'q' || k == 'Q')
      {
        serial->close_port();
        term.restore();
        std::cout << "\nSetup cancelled by user.\n";
        exit(0);
      }

      usleep(50000); // 50ms tick
      tick++;
    }

    if (!advance_port)
    {
      break;
    }
  }

  return false;
}

int main(int argc, char ** argv)
{
  std::signal(SIGINT, sigint_handler);

  int right_port_idx = -1;
  int left_port_idx = -1;
  bool force_interactive = false;

  std::vector<char*> filtered_argv;
  filtered_argv.push_back(argv[0]);

  for (int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i];
    bool is_custom_flag = false;

    if (arg == "-i" || arg == "--interactive")
    {
      force_interactive = true;
      is_custom_flag = true;
    }
    // Matches flags like -0, -1, -2, -3...
    else if (arg[0] == '-' && arg.size() > 1 && std::isdigit(arg[1]))
    {
      try
      {
        int idx = std::stoi(arg.substr(1));
        if (right_port_idx == -1) right_port_idx = idx;
        else if (left_port_idx == -1) left_port_idx = idx;
        is_custom_flag = true;
      }
      catch (...) {}
    }
    // Matches raw numbers like 0, 1, 2, 3...
    else if (std::isdigit(arg[0]))
    {
      try
      {
        int idx = std::stoi(arg);
        if (right_port_idx == -1) right_port_idx = idx;
        else if (left_port_idx == -1) left_port_idx = idx;
        is_custom_flag = true;
      }
      catch (...) {}
    }

    if (!is_custom_flag)
    {
      filtered_argv.push_back(argv[i]);
    }
  }

  // Parse arguments early (handles --help, --network, --namespace, etc.)
  int filtered_argc = filtered_argv.size();
  auto vm = param::helper(filtered_argc, filtered_argv.data());

  std::string final_left_port = "";
  int final_left_id = 1;
  std::string final_right_port = "";
  int final_right_id = 1;

  // Determine whether to run interactive discovery or direct command-line execution
  if (right_port_idx != -1 && !force_interactive)
  {
    // Non-interactive mode using provided ports
    final_right_port = "/dev/ttyUSB" + std::to_string(right_port_idx);
    if (left_port_idx != -1)
    {
      final_left_port = "/dev/ttyUSB" + std::to_string(left_port_idx);
    }
    std::cout << "Starting in non-interactive mode with specified ports:\n";
    std::cout << "  Right Hand : " << final_right_port << "\n";
    std::cout << "  Left Hand  : " << (final_left_port.empty() ? "Disabled" : final_left_port) << "\n";
  }
  else
  {
    // Interactive discovery mode
    std::cout << "\n============================================================\n";
    std::cout << "          Unitree G1 Inspire Hand Setup & Server\n";
    std::cout << "============================================================\n";

    RawTerminal term;
    g_raw_term = &term;

    std::vector<std::string> ports = getAvailablePorts();
    std::cout << "Candidate ports detected: ";
    for (size_t i = 0; i < ports.size(); ++i)
    {
      std::cout << ports[i] << (i + 1 < ports.size() ? ", " : "\n\n");
    }

    // Step 1: Discover LEFT HAND first
    std::vector<std::string> left_candidates = ports;
    testAndSelectHand("LEFT", left_candidates, term, final_left_port, final_left_id);

    // Step 2: Discover RIGHT HAND next (excluding port chosen for left hand)
    std::vector<std::string> right_candidates;
    for (const auto& p : ports)
    {
      if (p != final_left_port)
      {
        right_candidates.push_back(p);
      }
    }
    testAndSelectHand("RIGHT", right_candidates, term, final_right_port, final_right_id);

    term.restore();
    g_raw_term = nullptr;

    if (final_left_port.empty() && final_right_port.empty())
    {
      std::cerr << "Both hands were skipped. Exiting server.\n";
      return 0;
    }
  }

  // Initialize DDS and run
  unitree::robot::ChannelFactory::Instance()->Init(0, param::network);

  std::cout << "============================================================\n";
  std::cout << "          Unitree Robotics Inspire Hand Controller\n";
  std::cout << "============================================================\n";
  std::cout << " Active configuration:\n";
  std::cout << "   Left Hand : " << (final_left_port.empty() ? "Disabled" : (final_left_port + " (ID " + std::to_string(final_left_id) + ")")) << "\n";
  std::cout << "   Right Hand: " << (final_right_port.empty() ? "Disabled" : (final_right_port + " (ID " + std::to_string(final_right_id) + ")")) << "\n";
  std::cout << " DDS Topics  : rt/" << param::ns << "/cmd , rt/" << param::ns << "/state\n";
  std::cout << " Ready. Press Ctrl+C to terminate.\n";
  std::cout << "============================================================\n\n";

  InspireRunner runner(final_right_port, final_right_id, final_left_port, final_left_id);

  while (!g_interrupted)
  {
    sleep(1);
  }

  return 0;
}