#ifndef TCS_HPP
#define TCS_HPP
#include "../component/xml/pugixml.hpp"
#include "../kernel/dispatch/dispatch.hpp"
#include "../kernel/schedule/schedule.hpp"
#ifdef VISUAL
#include "../component/visualization/visualizer.hpp"
#endif

class TCS : public std::enable_shared_from_this<TCS> {
 public:
  bool init_all(const std::string& xml_path, double r = 30);
  bool init_resource(const std::string& xml_path);
  bool init_dispatcher();
  bool init_scheduler();
  bool init_orderpool();

  void run();
  ~TCS();

 public:
#ifdef VISUAL
  std::shared_ptr<visual::Visualizer> visualizer;
  bool init_visualizer(double);
#endif
 public:
  std::shared_ptr<kernel::allocate::ResourceManager> resource;
  std::shared_ptr<kernel::schedule::Scheduler> scheduler;
  std::shared_ptr<kernel::dispatch::Dispatcher> dispatcher;
  std::shared_ptr<kernel::allocate::OrderPool> orderpool;
};
#endif