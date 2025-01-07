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
tchecker::parsing::system_declaration_t * load_system_declaration(std::string const & filename);
void writeInitialData(std::ofstream & file);
void writeSystemClocks(std::ofstream & file, std::vector<std::string> & system_clocks,
                       const std::shared_ptr<tchecker::parsing::system_declaration_t> & property_decl);
void writeIntegerVariables(std::ofstream & file, const std::string & input_file);
tchecker::node_id_t assignNodeIDs(std::map<node_sptr_t, tchecker::node_id_t, node_lexical_less_t> & nodes_map,
                                  const graph_t & graph);
void writeNodeLocations(std::ofstream & file, const graph_t & graph,
                        const std::map<node_sptr_t, tchecker::node_id_t, node_lexical_less_t> & nodes_map);
void writeEdges(std::ofstream & file, const graph_t & graph,
                const std::map<node_sptr_t, tchecker::node_id_t, node_lexical_less_t> & nodes_map,
                std::set<std::string> & declared_events);
void writeEnvironmentData(std::ofstream & file, const std::string & env_file, std::set<std::string> & declared_events,
                          std::map<std::string, std::set<std::string>> & events_per_ps);

void writeNodeAttributes(std::ofstream & file, const std::map<std::string, std::string> & attr, const node_sptr_t & node,
                         const tchecker::system::system_t & graph_system);
void writeEdge(std::ofstream & file, tchecker::node_id_t src, tchecker::node_id_t tgt, const edge_sptr_t & edge,
               const std::map<std::string, std::string> & attr, std::set<std::string> & declared_events,
               tchecker::system::system_t graph_system);
void writeSynchronization(std::ofstream & file, const std::set<std::string> & declared_events,
                          const std::map<std::string, std::set<std::string>> & events_per_ps);

tchecker::parsing::system_declaration_t *
graph_parser(const std::string & input_file, const std::string & env_file,
             const std::shared_ptr<tchecker::parsing::system_declaration_t> & property_decl, const graph_t & graph,
             std::ostream * os, uint32_t & nodes_count)
{
  std::ofstream m_file("m_file.txt");

  // Step 1: Write header system data
  writeInitialData(m_file);

  // Step 2: Write system clocks
  std::vector<std::string> system_clocks;
  writeSystemClocks(m_file, system_clocks, property_decl);

  // Step 3: Declare bounded integer variables
  writeIntegerVariables(m_file, input_file);

  // Step 4: Assign each node a unique ID
  std::map<node_sptr_t, tchecker::node_id_t, node_lexical_less_t> nodes_map;
  nodes_count = assignNodeIDs(nodes_map, graph);

  // Step 5: Write node locations with attributes
  writeNodeLocations(m_file, graph, nodes_map);

  // Step 6: Write edges based on node IDs
  std::set<std::string> declared_events;
  writeEdges(m_file, graph, nodes_map, declared_events);

  // Step 7: Write environment data
  std::map<std::string, std::set<std::string>> events_per_ps;
  writeEnvironmentData(m_file, env_file, declared_events, events_per_ps);

  // Step 8: Write synchronization data
  writeSynchronization(m_file, declared_events, events_per_ps);

  m_file.close();

  std::ifstream read_file("m_file.txt");
  if (read_file.is_open()) {
    std::string line;
    while (std::getline(read_file, line)) {
      *os << line << std::endl;
    }
  }
  read_file.close();

//  *os << "BACKWARD_REACH_STATES " << nodes_count << "\n\n";
  return load_system_declaration("m_file.txt");
}

void writeInitialData(std::ofstream & file)
{
  file << "system:mergedSystem\n\n";
  file << "process:sys\n\n";
}

void writeSystemClocks(std::ofstream & file, std::vector<std::string> & system_clocks,
                       const std::shared_ptr<tchecker::parsing::system_declaration_t> & property_decl)
{
  for (const auto & [key, _] : property_decl->get_clocks()) {
    file << "clock:1:" << key << "\n";
    system_clocks.push_back(key);
  }
  file << "\n";
}

void writeIntegerVariables(std::ofstream & file, const std::string & input_file)
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

void writeNodeLocations(std::ofstream & file, const graph_t & graph,
                        const std::map<node_sptr_t, tchecker::node_id_t, node_lexical_less_t> & nodes_map)
{
  auto graph_system = graph->zg().system().as_system_system();
  for (const auto & [node, id] : nodes_map) {
    file << "location:sys:S" << id << "{";

    std::map<std::string, std::string> attr;
    graph->attributes(node, attr);
    writeNodeAttributes(file, attr, node, graph_system);

    file << "}\n";

    // auto reset_vector = node->reset_history_vector();
    // file << reset_vector << std::endl;
  }
  file << "\n";
}

void writeNodeAttributes(std::ofstream & file, const std::map<std::string, std::string> & attr, const node_sptr_t & node,
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
      if (inv_printed == false) {
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
        if (inv_printed == false) {
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

void writeEdges(std::ofstream & file, const graph_t & graph,
                const std::map<node_sptr_t, tchecker::node_id_t, node_lexical_less_t> & nodes_map,
                std::set<std::string> & declared_events)
{
  std::multiset<extended_edge_t, extended_edge_le_t> edges_set;

  for (const auto & node : graph->nodes()) {
    for (const auto & edge : graph->outgoing_edges(node)) {
      if (graph->edge_src(edge)->get_reach_status() && graph->edge_tgt(edge)->get_reach_status()) {
        auto src_it = nodes_map.find(graph->edge_src(edge));
        auto tgt_it = nodes_map.find(graph->edge_tgt(edge));

        if (src_it != nodes_map.end() && tgt_it != nodes_map.end()) {
          edges_set.emplace(src_it->second, tgt_it->second, edge);
        }
        else {
          throw std::runtime_error("Edge source or target not found in node map.");
        }
      }
    }
  }

  std::map<std::string, std::string> attr;
  auto graph_system = graph->zg().system().as_system_system();

  for (const auto & [src, tgt, edge] : edges_set) {
    attr.clear();
    graph->attributes(edge, attr);
    writeEdge(file, src, tgt, edge, attr, declared_events, graph_system);
  }
}

void writeEdge(std::ofstream & file, tchecker::node_id_t src, tchecker::node_id_t tgt, const edge_sptr_t & edge,
               const std::map<std::string, std::string> & attr, std::set<std::string> & declared_events,
               tchecker::system::system_t graph_system)
{
  // Custom function to handle edge attributes (e.g., event, do/provided statements)

  for (const auto & [key, value] : attr) {
    if (key == "vedge") {
      std::basic_string<char>::size_type startIndex = value.find('@');
      std::basic_string<char>::size_type endIndex = value.find(',');
      // Determine the substring based on the presence of endIndex
      std::string sub_string = (endIndex != std::string::npos) ? value.substr(startIndex + 1, (endIndex - 1) - startIndex)
                                                               : value.substr(startIndex + 1, value.length() - startIndex - 2);

      bool insert_line = declared_events.insert(sub_string).second;
      // It will search and insert if the event wasn't previously declared
      if (insert_line) {
        file << "\nevent:" << sub_string << "\n";
      }
      // print edge
      file << "edge:sys:S" << std::to_string(src) << ":S" << std::to_string(tgt) << ":" << sub_string << "{";
      // check for do statements
      bool do_printed = false;
      for (auto edge_id : edge->vedge()) {
        // do statements
        auto single_edge = graph_system.edge(edge_id);
        auto const & edge_range = single_edge->attributes().range("do");
        if (!edge_range.empty()) {
          // if there exists at least one do statement
          if (do_printed == false) {
            file << "do:";
          }
          for (auto edge_it = edge_range.begin(); edge_it != edge_range.end();) {
            auto const clock_assignment = *edge_it;
            if (do_printed == false) {
              file << clock_assignment.value();
              do_printed = true;
            }
            else {
              file << ";" << clock_assignment.value();
            }
            if (++edge_it != edge_range.end()) {
              file << ";";
            }
          }
        }
      }
      // check for provided statements
      bool provided_printed = false;
      for (auto edge_id : edge->vedge()) {
        // provided statements
        auto single_edge = graph_system.edge(edge_id);
        auto const & edge_range = single_edge->attributes().range("provided");
        if (!edge_range.empty()) {
          // if there exists at least one provided statement
          if (do_printed == true) {
            // if there exist a do statement before
            file << ":";
            do_printed = false; // prohibit second run
          }
          if (provided_printed == false) {
            // if provided was not printed before
            file << "provided:";
          }
          for (auto edge_it = edge_range.begin(); edge_it != edge_range.end();) {
            auto const clock_condition = *edge_it;
            if (provided_printed == false) {
              file << clock_condition.value();
              provided_printed = true;
            }
            else {
              file << " && " << clock_condition.value();
            }
            if (++edge_it != edge_range.end()) {
              file << " && ";
            }
          }
        }
      }
      file << "}\n";
      if (insert_line) {
        // insert another line for convenience
        file << "\n";
      }
    }
  }
}

void writeEnvironmentData(std::ofstream & file, const std::string & env_file, std::set<std::string> & declared_events,
                          std::map<std::string, std::set<std::string>> & events_per_ps)
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
      // add to declared environment processes // if the ps was not previously added (just to be sure, should not happen)
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
      for (const std::string & cur_event : declared_events) {
        // we only need to check shared events between system and env, for the purpose of sync later
        if (line.find(cur_event) != std::string::npos) {
          the_event = cur_event;
        }
      }

      events_per_ps[the_ps].insert(the_event);
      // add the event to the corresponding list (will create the set if it's the first event to be added)
    }
    else if (line.find("clock") != std::string::npos) {
      file << line << "\n\n";
      std::basic_string<char>::size_type start_index = line.find_last_of(':');
      std::string clock_name = line.substr(start_index + 1, line.length() - start_index - 1);
      env_clocks.push_back(clock_name); // add the new clock to the list of env clocks
    }
    else if (line.find("sync") != std::string::npos) {
      file << line << "\n\n";
    }
  }
}

void writeSynchronization(std::ofstream & file, const std::set<std::string> & declared_events,
                          const std::map<std::string, std::set<std::string>> & events_per_ps)
{
  // Write synchronization lines based on declared events and events per process
  for (std::string event : declared_events) {
    if (event[0] != '_') {
      // if not epsilon
      file << "sync:"
           << "sys@" << event;
      for (const auto & pair : events_per_ps) {
        std::string ps = pair.first;
        for (const auto & found_event : pair.second) {
          if (found_event == event) {
            file << ":" << ps << "@" << event;
          }
        }
      }
    }
    file << "\n";
  }
}

tchecker::parsing::system_declaration_t * load_system_declaration(std::string const & filename)
{
  tchecker::parsing::system_declaration_t * sysdecl = nullptr;
  try {
    sysdecl = tchecker::parsing::parse_system_declaration(filename);
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