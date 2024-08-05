#ifndef RSSRESOURCE_HPP
#define RSSRESOURCE_HPP
#include "rssobject.hpp"
class RSSResource : public RSSObject {
 public:
  virtual ~RSSResource() = default;
  using RSSObject::RSSObject;
  std::weak_ptr<RSSObject> owner;
  std::vector<std::weak_ptr<RSSObject>> future_owner;
  std::map<std::string, std::shared_ptr<RSSObject>> envelopes;
  std::set<std::string> envelope_keys;
};
#endif