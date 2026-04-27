/*
 * See files AUTHORS and LICENSE for copyright details.
 */

#ifndef PARSE_GRAPH_HH
#define PARSE_GRAPH_HH

#include <fstream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "tchecker/parsing/parsing.hh"
#include "zg-history-aware.hh"

namespace tchecker {

namespace tck_reach {

// Type aliases for clarity
using graph_t = typename std::shared_ptr<tchecker::tck_reach::zg_history_aware::graph_t>;
using node_sptr_t = typename tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t;
using edge_sptr_t = typename tchecker::tck_reach::zg_history_aware::graph_t::edge_sptr_t;
using node_lexical_less_t = typename tchecker::tck_reach::zg_history_aware::node_lexical_less_t;
using extended_edge_t = typename std::tuple<tchecker::node_id_t, tchecker::node_id_t, edge_sptr_t>; // <src, tgt, edge_sptr>

// Extend EDGE_LE on triples (src, tgt, edge)
class extended_edge_le_t : private tchecker::tck_reach::zg_history_aware::edge_lexical_less_t {
public:
  bool operator()(extended_edge_t const & e1, extended_edge_t const & e2) const
  {
    auto && [src1, tgt1, edge_sptr1] = e1;
    auto && [src2, tgt2, edge_sptr2] = e2;
    if (src1 != src2)
      return src1 < src2;
    if (tgt1 != tgt2)
      return tgt1 < tgt2;

    return tchecker::tck_reach::zg_history_aware::edge_lexical_less_t::operator()(edge_sptr1, edge_sptr2);
  }
};

// Function declarations
std::shared_ptr<tchecker::parsing::system_declaration_t> load_system_declaration(std::string const & filename);
void writeInitialData(std::ostream & file);
void writeSystemClocks(std::ostream & file, const std::string & input_file);
void writeIntegerVariables(std::ostream & file, const std::string & input_file);
tchecker::node_id_t assignNodeIDs(std::map<node_sptr_t, tchecker::node_id_t, node_lexical_less_t> & nodes_map,
                                  const graph_t & graph);
void writeNodeLocations(std::ostream & file, const graph_t & graph,
                        const std::map<node_sptr_t, tchecker::node_id_t, node_lexical_less_t> & nodes_map);
void writeEdges(std::ostream & file, const graph_t & graph,
                const std::map<node_sptr_t, tchecker::node_id_t, node_lexical_less_t> & nodes_map,
                std::set<std::string> & declared_events);
void writeEnvironmentData(std::ostream & file, const std::string & env_file,
                          const std::shared_ptr<tchecker::parsing::system_declaration_t> & property_decl,
                          std::set<std::string> & declared_events, std::map<std::string, std::set<std::string>> & events_per_ps);

void writeNodeAttributes(std::ostream & file, const std::map<std::string, std::string> & attr, const node_sptr_t & node,
                         const tchecker::system::system_t & graph_system);
void writeEdge(std::ostream & file, tchecker::node_id_t src, tchecker::node_id_t tgt, const edge_sptr_t & edge,
               const std::map<std::string, std::string> & attr, std::set<std::string> & declared_events,
               tchecker::system::system_t graph_system, std::unordered_map<std::string, std::string> & vedge_event_cache);
void writeSynchronization(std::ostream & file, std::istream & original_syncs, const std::set<std::string> & prop_processes, const std::set<std::string> & env_processes);

void graph_parser_prologue(std::ostringstream & buffer, const std::string & input_file)
{

  // Write header system data
  writeInitialData(buffer);

  // Write system and environment clocks
  std::vector<std::string> system_clocks;
  writeSystemClocks(buffer, input_file);

  // writeIntegerVariables(buffer, input_file); // this was intended for integer variable support and is not yet fully implemented:
}

 std::shared_ptr<tchecker::parsing::system_declaration_t> graph_parser_epilogue(std::ostringstream & buffer, const std::string & env_file,
             std::shared_ptr<tchecker::parsing::system_declaration_t> const & envdecl,
             const std::shared_ptr<tchecker::parsing::system_declaration_t> & property_decl,
             std::ostream * os, const graph_t & graph, std::set<std::string> declared_events,
             std::stringstream & original_syncs,
             std::set<std::string> & prop_processes,
             std::set<std::string> & env_processes)
{

  // Write environment data
  std::map<std::string, std::set<std::string>> events_per_ps;
  writeEnvironmentData(buffer, env_file, property_decl, declared_events, events_per_ps);

  // Write synchronization data
  writeSynchronization(buffer, original_syncs, prop_processes, env_processes);

  std::istringstream in(buffer.str());

  return load_system_declaration(buffer.str());
}

void writeInitialData(std::ostream & file)
{
  file << "system:mergedSystem\n\n";
  file << "process:sys\n\n";
}

void writeSystemClocks(std::ostream & file, const std::string & input_file)
{
  std::ifstream input_file_stream(input_file);
  std::string line;
  while (std::getline(input_file_stream, line)) {
    if (line.find("clock:") != std::string::npos) {
      file << line << "\n";
    }
  }
  file << "\n";
}

void writeIntegerVariables(std::ostream & file, const std::string & input_file)
{
  std::ifstream input_file_stream(input_file);
  std::string line;
  while (std::getline(input_file_stream, line)) {
    if (line.find("int") != std::string::npos) {
      file << line << "\n";
    }
  }
  file << "\n";
}

tchecker::node_id_t assignNodeIDs(std::map<node_sptr_t, tchecker::node_id_t, node_lexical_less_t> & nodes_map,
                                  const graph_t & graph)
{
  for (const auto & node : graph->nodes()) {
    if (node->get_reach_status()) {
      nodes_map.insert(std::make_pair(node, 0));
    }
  }

  tchecker::node_id_t node_count = 0;
  for (auto & [_, id] : nodes_map) {
    id = node_count++;
  }

  return node_count;
}

void writeNodeAttributes(std::ostream & file, const std::map<std::string, std::string> & attr, const node_sptr_t & node,
                         const tchecker::system::system_t & graph_system)
{
  // Custom function for writing node attributes (e.g., initial, final, invariants, etc.)
  bool pi_printed = false;
  for (const auto & [key, value] : attr) {
    if (key == "final") {
      // add pi label to finals
      file << "labels:Pi";
      pi_printed = true;
    }
  }

  // invariants
  bool inv_printed = false;
  for (auto loc_id : node->state_ptr()->vloc()) {
    auto single_location = graph_system.location(loc_id);
    const auto & loc_range = single_location->attributes().range("invariant");
    if (!loc_range.empty()) {
      // if there exists at least one invariant
      if (!inv_printed) {
        // if (pi_printed == true) {        // previous implementation
        //    m_file << ":";                // previous implementation
        // }                                // previous implementation
        if (!pi_printed) {
          // if not pi, otherwise we don't need the invariant
          file << "invariant:";
        }
        else {
          file << ":invariant:";
        }
      }
      for (auto loc_it = loc_range.begin(); loc_it != loc_range.end();) {
        auto const loc_invariant = *loc_it;
        if (!inv_printed) {
          file << loc_invariant.value();
          inv_printed = true;
        }
        else {
          file << " && " << loc_invariant.value();
        }
        if (++loc_it != loc_range.end()) {
          file << " && ";
        }
      }
    }
  }

  for (const auto & [key, value] : attr) {
    if (key == "initial") {
      if (inv_printed) {
        file << ":initial:";
      }
      else {
        file << "initial:";
      }
    }
  }
}

void writeEdge(std::ostream & file,
               unsigned long long int src,
               unsigned long long int tgt,
               const edge_sptr_t & edge,
               const std::map<std::string, std::string> & attr,
               std::set<std::string> & declared_events,
               tchecker::system::system_t graph_system,
               std::unordered_map<std::string, std::string> & vedge_event_cache,
               std::unordered_map<std::string, bool> & edge_presence_map)
{
  // find vedge attribute
  auto it = attr.find("vedge");
  if (it == attr.end())
    return;

  const std::string & vedge_value = it->second;

  // parse or reuse cached event name (preserve original substring behavior)
  std::string event_name;
  auto cache_it = vedge_event_cache.find(vedge_value);
  if (cache_it != vedge_event_cache.end()) {
    event_name = cache_it->second;
  }
  else {
    auto startIndex = vedge_value.find('@');
    auto endIndex = vedge_value.find(',');

    if (endIndex != std::string::npos)
      event_name = vedge_value.substr(startIndex + 1, (endIndex - 1) - startIndex);
    else
      event_name = vedge_value.substr(startIndex + 1,
                                       vedge_value.length() - startIndex - 2);

    vedge_event_cache.emplace(vedge_value, event_name);
  }

  // declare event if new
  bool insert_line = declared_events.insert(event_name).second;
  if (insert_line) {
    file << "\nevent:" << event_name << "\n";
  }

  // Collect all 'do' and all 'provided' values across the inner edges.
  std::vector<std::string> do_vals;
  std::vector<std::string> prov_vals;

  for (auto edge_id : edge->vedge()) {
    auto single_edge = graph_system.edge(edge_id);
    auto const & attrs = single_edge->attributes();

    // collect do values
    for (auto it2 = attrs.range("do").begin(); it2 != attrs.range("do").end(); ++it2) {
      do_vals.push_back((*it2).value());
    }

    // collect provided values
    for (auto it2 = attrs.range("provided").begin(); it2 != attrs.range("provided").end(); ++it2) {
      prov_vals.push_back((*it2).value());
    }
  }

  std::string aggr_dos;

  // Write aggregated DOs (if any)
  bool wrote_do = false;
  if (!do_vals.empty()) {
    wrote_do = true;
    aggr_dos += "do:";
    for (std::size_t i = 0; i < do_vals.size(); ++i) {
      if (i != 0)
        aggr_dos += ";";
      aggr_dos += do_vals[i];
    }
  }

  std::string aggr_provides;

  // Write aggregated PROVIDEDs (if any)
  if (!prov_vals.empty()) {
    // if there were do items, add colon separator between sections
    if (wrote_do) {
      aggr_provides += ":";
    }
    aggr_provides += "provided:";
    for (std::size_t i = 0; i < prov_vals.size(); ++i) {
      if (i != 0)
        aggr_provides += " && ";
      aggr_provides += prov_vals[i];
    }
  }

  std::string aggr_line;
  aggr_line.reserve(256);  // estimate size

  aggr_line.append("edge:sys:S");
  aggr_line.append(std::to_string(src));
  aggr_line.append(":S");
  aggr_line.append(std::to_string(tgt));
  aggr_line.append(":");
  aggr_line.append(event_name);
  aggr_line.append("{");
  aggr_line.append(aggr_dos);
  aggr_line.append(aggr_provides);
  aggr_line.append("}");

  if (edge_presence_map.find(aggr_line) == edge_presence_map.end()) {
    edge_presence_map[aggr_line] = true;
    // Write the edge
    file << aggr_line << "\n";
    // optionally add blank line after event declaration
    if (insert_line)
      file << "\n";
  }
}

void writeEnvironmentData(std::ostream & file, const std::string & env_file,
                          const std::shared_ptr<tchecker::parsing::system_declaration_t> & property_decl,
                          std::set<std::string> & declared_events, std::map<std::string, std::set<std::string>> & events_per_ps)
{
  std::vector<std::string> env_clocks;
  std::set<std::string> declared_asynch_events;
  std::set<std::string> declared_env_ps;

  std::ifstream env_file_stream(env_file);
  std::string line;
  while (std::getline(env_file_stream, line)) {
    if (line[0] == '#')
      continue; // if the line is a comment

    if (line.find("process") != std::string::npos) {
      file << "\n" << line << "\n\n";
      std::basic_string<char>::size_type index = line.find(':');
      std::string ps = line.substr(index + 1, line.length() - index - 1);
      file << "# ps name: " << ps << "\n\n";
      declared_env_ps.insert(ps);
      // add to declared environment processes
    }
    else if (line.find("location") != std::string::npos) {
      file << line << "\n\n";
    }
    else if (line.find("event") != std::string::npos) {
      std::basic_string<char>::size_type start_index = line.find(':');
      std::string events_name = line.substr(start_index + 1, line.length() - start_index - 1);
      if (declared_events.find(events_name) == declared_events.end()) {
        // if not prev. declared in system events
        if (declared_asynch_events.insert(events_name).second) {
          // if not prev. declared in env events
          file << "event:" << events_name << "\n\n";
        }
      }
    }
    else if (line.find("edge") != std::string::npos) {
      file << line << "\n\n";
      std::string the_ps;
      for (const std::string & cur_ps : declared_env_ps) {
        // find the corresponding process
        if (line.find(cur_ps) != std::string::npos) {
          the_ps = cur_ps; // note that processes should be declared before events, otherwise ps may not be initialized
          break;
        }
      }
      std::string the_event;
      for (const auto & [cur_event, _] : property_decl->get_events()) {
        // we only need to check shared events between system and env, for the purpose of sync later
        if (line.find(cur_event) != std::string::npos) {
          the_event = cur_event;
          break;
        }
      }

      if (!the_event.empty()) {
        events_per_ps[the_ps].insert(the_event);
        // add the event to the corresponding list
      }
    }
  }
}

void writeSynchronization(
    std::ostream & file,
    std::istream & original_syncs,
    const std::set<std::string> & prop_processes,
    const std::set<std::string> & env_processes)
{
  std::string line;

  while (std::getline(original_syncs, line)) {

    // Ignore empty or non-sync lines
    if (line.empty() || line.rfind("sync:", 0) != 0)
      continue;

    // Remove "sync:"
    std::string rest = line.substr(5);

    std::vector<std::string> prop_parts;
    std::vector<std::string> env_parts;

    std::string event_name;

    std::stringstream ss(rest);
    std::string token;

    // Split by ':'
    while (std::getline(ss, token, ':')) {

      auto at_pos = token.find('@');
      if (at_pos == std::string::npos)
        continue;

      std::string proc = token.substr(0, at_pos);
      std::string event = token.substr(at_pos + 1);

      if (event_name.empty())
        event_name = event;

      if (prop_processes.count(proc)) {
        prop_parts.push_back(proc);
      }
      else if (env_processes.count(proc)) {
        env_parts.push_back(proc);
      }
      else {
        // default: treat unknown process as env
        env_parts.push_back(proc);
      }
    }

    // Rule 1: all in prop : omit
    if (!prop_parts.empty() && env_parts.empty()) {
      continue;
    }

    // Rule 2: all in env : write unchanged
    if (prop_parts.empty() && !env_parts.empty()) {
      file << line << "\n";
      continue;
    }

    // Rule 3: mixed : collapse prop into sys@event
    if (!prop_parts.empty() && !env_parts.empty()) {

      file << "sync";

      // write env participants
      for (const auto & ps : env_parts) {
        file << ":" << ps << "@" << event_name;
      }

      // merged prop side
      file << ":sys@" << event_name;

      file << "\n";
    }
  }
}

std::shared_ptr<tchecker::parsing::system_declaration_t> load_system_declaration(std::string const & file)
{
  std::shared_ptr<tchecker::parsing::system_declaration_t> sysdecl{nullptr};
  try {
    sysdecl = tchecker::parsing::parse_system_declaration_from_buffer(file, "in_memory");
    if (sysdecl == nullptr)
      throw std::runtime_error("nullptr system declaration");
  }
  catch (std::exception const & e) {
    std::cerr << tchecker::log_error << e.what() << std::endl;
  }
  return sysdecl;
}

} // namespace tck_reach

} // namespace tchecker

#endif // PARSE_GRAPH_HH