#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>

namespace auto_vehicle
{

// Cycles a gazebo_traffic_light model through a configurable sequence of
// phases by resetting its red_switch / green_switch / yellow_switch /
// right_turn_switch prismatic joints between their 0 (off) and 0.101 (on)
// limits. Each phase is one <phase duration="seconds" color="red|yellow|
// green|right_turn|off"/> element in the <plugin> block; phases repeat in
// order, looping over the sum of all durations. A single phase keeps that
// color on permanently.
//
// Can be attached directly to a gazebo_traffic_light model, or to a wrapper
// model (e.g. cantilevered_light) that contains a nested model named
// "traffic_light", so each instance in a world can get its own schedule.
class TrafficLightPlugin
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{
  private: struct Phase
  {
    double duration;
    std::string color;
  };

  public: void Configure(
      const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager &/*_eventMgr*/) override
  {
    ignition::gazebo::Model model(_entity);

    this->redJoint = model.JointByName(_ecm, "red_switch");
    if (this->redJoint == ignition::gazebo::kNullEntity)
    {
      for (const auto &child : model.Models(_ecm))
      {
        if (ignition::gazebo::Model(child).Name(_ecm) == "traffic_light")
        {
          model = ignition::gazebo::Model(child);
          break;
        }
      }
      this->redJoint = model.JointByName(_ecm, "red_switch");
    }
    this->yellowJoint = model.JointByName(_ecm, "yellow_switch");
    this->greenJoint = model.JointByName(_ecm, "green_switch");
    this->rightTurnJoint = model.JointByName(_ecm, "right_turn_switch");

    for (auto phaseElem = _sdf->FindElement("phase"); phaseElem;
        phaseElem = phaseElem->GetNextElement("phase"))
    {
      Phase phase;
      phase.duration = phaseElem->Get<double>("duration", 5.0).first;
      phase.color = phaseElem->Get<std::string>("color", "off").first;
      this->cycleDuration += phase.duration;
      this->phases.push_back(phase);
    }
  }

  public: void PreUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (_info.paused || this->phases.empty() || this->cycleDuration <= 0.0)
      return;

    // Use simulation time (not wall-clock time) so the schedule tracks
    // Gazebo's clock: it holds still while paused and scales with
    // real-time-factor instead of drifting against it.
    const double simSeconds =
        std::chrono::duration<double>(_info.simTime).count();

    double t = std::fmod(simSeconds, this->cycleDuration);

    std::string activeColor = this->phases.front().color;
    for (const auto &phase : this->phases)
    {
      if (t < phase.duration)
      {
        activeColor = phase.color;
        break;
      }
      t -= phase.duration;
    }

    this->SetOn(_ecm, this->redJoint, activeColor == "red");
    this->SetOn(_ecm, this->greenJoint, activeColor == "green");
    this->SetOn(_ecm, this->yellowJoint, activeColor == "yellow");
    this->SetOn(_ecm, this->rightTurnJoint, activeColor == "right_turn");
  }

  private: void SetOn(ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::Entity _joint, bool _on)
  {
    if (_joint == ignition::gazebo::kNullEntity)
      return;
    ignition::gazebo::Joint joint(_joint);
    joint.ResetPosition(_ecm, {_on ? 0.101 : 0.0});
  }

  private: ignition::gazebo::Entity redJoint{ignition::gazebo::kNullEntity};
  private: ignition::gazebo::Entity yellowJoint{ignition::gazebo::kNullEntity};
  private: ignition::gazebo::Entity greenJoint{ignition::gazebo::kNullEntity};
  private: ignition::gazebo::Entity rightTurnJoint{
      ignition::gazebo::kNullEntity};

  private: std::vector<Phase> phases;
  private: double cycleDuration{0.0};
};

}  // namespace auto_vehicle

IGNITION_ADD_PLUGIN(
    auto_vehicle::TrafficLightPlugin,
    ignition::gazebo::System,
    auto_vehicle::TrafficLightPlugin::ISystemConfigure,
    auto_vehicle::TrafficLightPlugin::ISystemPreUpdate)
