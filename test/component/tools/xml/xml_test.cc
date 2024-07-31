#include <iostream>

#include "../../../../include/3rdparty/utest/utest.h"
#include "../../../../include/3rdparty/xml/pugixml.hpp"

UTEST(load_file, case1) {
  pugi::xml_document doc;
  auto ret = doc.load_file("xml_test.xml");
  EXPECT_EQ(pugi::xml_parse_status::status_ok, ret.status);
  //   std::cout << ret.description() << "\n";
  //   doc.save(std::cout);
}
UTEST(load_file, case2) {
  pugi::xml_document doc;
  auto ret = doc.load_file("xml_test.xml");
  EXPECT_STREQ(doc.first_child().name(), "robot");

  auto p = doc.first_child().first_child().find_child(
      [](pugi::xml_node node) { return std::string(node.name()) == "point"; });
  EXPECT_TRUE(p.type() == pugi::xml_node_type::node_null);
  EXPECT_STREQ(p.attribute("name").as_string(), "p1");
  while (p.type() != pugi::xml_node_type::node_null) {
    if (std::string(p.name()) != "point") {
      break;
    }
    std::cout << p.child("gcc").text() << "\n";
    std::cout << p.attribute("name").as_string() << "\n";
    p = p.next_sibling();
  }
}
UTEST_MAIN();
