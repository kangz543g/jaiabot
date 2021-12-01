#ifndef JAIABOT_SRC_BIN_MISSION_MANAGER_MACHINE_COMMON_H
#define JAIABOT_SRC_BIN_MISSION_MANAGER_MACHINE_COMMON_H

#include <goby/zeromq/application/multi_thread.h>

namespace jaiabot
{
namespace apps
{
class MissionManager;
}
namespace config
{
class MissionManager;
}

namespace statechart
{
struct MissionManagerStateMachine;

// provides access to parent App's methods (e.g. interthread() and interprocess()) from within the states' structs
template <typename Derived> class AppMethodsAccess
{
  protected:
    goby::zeromq::InterProcessPortal<goby::middleware::InterThreadTransporter>& interprocess()
    {
        return app().interprocess();
    }

    goby::middleware::InterThreadTransporter& interthread() { return app().interthread(); }

    const apps::MissionManager& app() const
    {
        return static_cast<const Derived*>(this)->outermost_context().app();
    }

    const config::MissionManager& cfg() const { return this->app().cfg(); }

    MissionManagerStateMachine& machine()
    {
        return static_cast<Derived*>(this)->outermost_context();
    }
    apps::MissionManager& app() { return machine().app(); }
};
} // namespace statechart
} // namespace jaiabot

#endif