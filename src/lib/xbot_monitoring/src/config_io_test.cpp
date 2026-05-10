// Standalone smoke test for config_io. Compile with:
//   g++ -std=c++17 -I../json/include config_io.cpp config_io_test.cpp -o config_io_test
// Run from this directory and pass the path to mower_config.schema.json as argv[1].

#include "config_io.h"

#include <cassert>
#include <iostream>
#include <stdexcept>

using json = nlohmann::ordered_json;
using namespace xbot_monitoring::config_io;

static void test_parse_and_write_round_trip() {
  std::string original =
      "# header comment\n"
      "\n"
      "export OM_FOO=\"hello world\"\n"
      "OM_BAR=42  # inline comment\n"
      "export OM_BAZ='single quoted'\n"
      "# another comment\n";

  json parsed = parse_config_sh(original);
  assert(parsed["OM_FOO"] == "hello world");
  assert(parsed["OM_BAR"] == "42");
  assert(parsed["OM_BAZ"] == "single quoted");

  json changes = json::object();
  changes["OM_FOO"] = "changed";
  changes["OM_NEW"] = "fresh";

  std::string updated = write_config_sh(original, changes);
  assert(updated.find("# header comment") != std::string::npos);
  assert(updated.find("OM_FOO=\"changed\"") != std::string::npos);
  assert(updated.find("OM_BAR=42") != std::string::npos);
  assert(updated.find("OM_BAZ='single quoted'") != std::string::npos ||
         updated.find("OM_BAZ=\"single quoted\"") != std::string::npos);
  assert(updated.find("OM_NEW=\"fresh\"") != std::string::npos);
  std::cout << "parse_and_write_round_trip OK" << std::endl;
}

static void test_boolean_value() {
  std::string original = "export OM_ENABLED=False\n";
  json changes = json::object();
  changes["OM_ENABLED"] = true;
  std::string updated = write_config_sh(original, changes);
  assert(updated.find("OM_ENABLED=\"True\"") != std::string::npos);
  std::cout << "boolean_value OK" << std::endl;
}

static void test_defaults_from_schema(const std::string& schema_path) {
  std::string raw = read_text_file(schema_path);
  json schema = json::parse(raw);
  json result = defaults_yaml_from_schema(schema);
  assert(result.contains("defaults.yaml"));
  std::string yaml = result["defaults.yaml"];
  assert(!yaml.empty());
  std::cout << "defaults_from_schema OK (yaml " << yaml.size() << " bytes)" << std::endl;
}

int main(int argc, char** argv) {
  test_parse_and_write_round_trip();
  test_boolean_value();
  if (argc >= 2) {
    test_defaults_from_schema(argv[1]);
  }
  std::cout << "all good" << std::endl;
  return 0;
}
