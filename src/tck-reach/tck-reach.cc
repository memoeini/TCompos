/*
 * This file is part of the TCompos tool, which builds upon the TChecker framework.
 *
 * See files AUTHORS and LICENSE for copyright details.
 *
 */

#include <fstream>
#include <getopt.h>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <queue>

#include "tchecker/algorithms/reach/algorithm.hh"
#include "tchecker/parsing/parsing.hh"
#include "tchecker/utils/log.hh"
#include "zg-reach.hh"

#include "parse-graph.hh"
#include "zg-reach-compos.hh"
#include "zg-history-aware.hh"

/*!
 \file tck-reach.cc
 \brief Reachability analysis of timed automata
 */

static struct option long_options[] = {{"algorithm", required_argument, 0, 'a'},
                                       {"certificate", required_argument, 0, 'C'},
                                       {"output", required_argument, 0, 'o'},
                                       {"help", no_argument, 0, 'h'},
                                       {"labels", required_argument, 0, 'l'},
                                       {"search-order", no_argument, 0, 's'},
                                       {"block-size", required_argument, 0, 0},
                                       {"table-size", required_argument, 0, 0},
                                       {"property-file", required_argument, 0, 'P'},
                                       {"env-file", required_argument, 0, 'E'},
                                       {"iterative", no_argument, 0, 'i'},
                                       {"merge-flag", no_argument, 0, 'm'},
                                       {0, 0, 0, 0}};

static char const * const options = (char *)"a:C:hl:o:s:P:E:m:i";

/*!
  \brief Display usage
  \param progname : programme name
*/
void usage(char * progname)
{
  std::cerr << "Usage: " << progname << " [options] [file]" << std::endl;
  std::cerr << "   -a algorithm  reachability algorithm" << std::endl;
  std::cerr << "          reach      standard reachability algorithm over the zone graph" << std::endl;
  std::cerr << "          compos   compositional reachability algorithm over the history aware zone graph" << std::endl;
  std::cerr << "   -C type       type of certificate (only for reach at the moment)" << std::endl;
  std::cerr << "          none       no certificate (default)" << std::endl;
  std::cerr << "          graph      graph of explored state-space" << std::endl;
  std::cerr << "          symbolic   symbolic run to a state with searched labels if any" << std::endl;
  std::cerr << "          concrete   concrete run to a state with searched labels if any" << std::endl;
  std::cerr << "   -h            help" << std::endl;
  std::cerr << "   -l l1,l2,...  comma-separated list of searched labels" << std::endl;
  std::cerr << "   -o out_file   output file for certificate (default is standard output)" << std::endl;
  std::cerr << "   -s bfs|dfs    search order" << std::endl;
  std::cerr << "   --block-size  size of allocation blocks" << std::endl;
  std::cerr << "   --table-size  size of hash tables" << std::endl;
  std::cerr << "reads from standard input if file is not provided" << std::endl;
}

enum algorithm_t {
  ALGO_REACH,    /*!< Reachability algorithm */
  ALGO_COMPOS,   /*!< Compositional algorithm */
  ALGO_NONE,     /*!< No algorithm */
};

enum certificate_t {
  CERTIFICATE_GRAPH,    /*!< Graph of state-space */
  CERTIFICATE_SYMBOLIC, /*!< Symbolic counter-example */
  CERTIFICATE_CONCRETE, /*!< Concrete counter-example */
  CERTIFICATE_NONE,     /*!< No certificate */
};

static enum algorithm_t algorithm = ALGO_NONE;            /*!< Selected algorithm */
static bool help = false;                                 /*!< Help flag */
static enum certificate_t certificate = CERTIFICATE_NONE; /*!< Type of certificate */
static std::string search_order = "bfs";                  /*!< Search order */
static std::string labels = "";                           /*!< Searched labels */
static std::string output_file = "";                      /*!< Output file name (empty means standard output) */
static std::ostream * os = &std::cout;                    /*!< Default output stream */
static std::size_t block_size = 10000;                    /*!< Size of allocated blocks */
static std::size_t table_size = 65536;                    /*!< Size of hash tables */
static std::string property_file = "";
static std::string env_file = "";
static bool early_enabled = false;
static bool merge_flag = false;

/*!
 \brief Parse command-line arguments
 \param argc : number of arguments
 \param argv : array of arguments
 \pre argv[0] up to argv[argc-1] are valid accesses
 \post global variables help, output_file, search_order and labels have been set
 from argv
*/
int parse_command_line(int argc, char * argv[])
{
  while (true) {
    int long_option_index = -1;
    int c = getopt_long(argc, argv, options, long_options, &long_option_index);

    if (c == -1)
      break;

    if (c == ':')
      throw std::runtime_error("Missing option parameter");
    else if (c == '?')
      throw std::runtime_error("Unknown command-line option");
    else if (c != 0) {
      switch (c) {
      case 'a':
        if (strcmp(optarg, "reach") == 0)
          algorithm = ALGO_REACH;
        else if (strcmp(optarg, "compos") == 0)
          algorithm = ALGO_COMPOS;
        else
          throw std::runtime_error("Unknown algorithm: " + std::string(optarg));
        break;
      case 'C':
        if (strcmp(optarg, "none") == 0)
          certificate = CERTIFICATE_NONE;
        else if (strcmp(optarg, "graph") == 0)
          certificate = CERTIFICATE_GRAPH;
        else if (strcmp(optarg, "concrete") == 0)
          certificate = CERTIFICATE_CONCRETE;
        else if (strcmp(optarg, "symbolic") == 0)
          certificate = CERTIFICATE_SYMBOLIC;
        else
          throw std::runtime_error("Unknown type of certificate: " + std::string(optarg));
        break;
      case 'E':
        env_file = optarg;
        break;
      case 'i':
        early_enabled = true;
        break;
      case 'm':
        merge_flag = true;
        break;
      case 'o':
        output_file = optarg;
        break;
      case 'P':
        property_file = optarg;
        break;
      case 'h':
        help = true;
        break;
      case 'l':
        labels = optarg;
        break;
      case 's':
        search_order = optarg;
        break;
      default:
        throw std::runtime_error("This should never be executed");
        break;
      }
    }
    else {
      if (strcmp(long_options[long_option_index].name, "block-size") == 0)
        block_size = std::strtoull(optarg, nullptr, 10);
      else if (strcmp(long_options[long_option_index].name, "table-size") == 0)
        table_size = std::strtoull(optarg, nullptr, 10);
      else
        throw std::runtime_error("This also should never be executed");
    }
  }

  return optind;
}

/*!
 \brief Load a system declaration from a file
 \param filename : file name
 \return pointer to a system declaration loaded from filename, nullptr in case
 of errors
 \post all errors have been reported to std::cerr
*/
std::shared_ptr<tchecker::parsing::system_declaration_t> load_system_declaration(std::string const & filename)
{
  std::shared_ptr<tchecker::parsing::system_declaration_t> sysdecl{nullptr};
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

/*!
 \brief Perform reachability analysis
 \param sysdecl : system declaration
 \post statistics on reachability analysis of command-line specified labels in
 the system declared by sysdecl have been output to standard output.
 A certification has been output if required.
*/
void reach(tchecker::parsing::system_declaration_t const & sysdecl)
{
  auto && [stats, graph] = tchecker::tck_reach::zg_reach::run(sysdecl, labels, search_order, block_size, table_size);

  // stats
  std::map<std::string, std::string> m;
  stats.attributes(m);
  for (auto && [key, value] : m)
    if (key == "REACHABLE") {
      if (value == "true") {
        std::cout << std::endl << "REACHABLE: true (Property VIOLATED)" << std::endl;
      } else {
        std::cout << std::endl << "REACHABLE: false (Property SATISFIED)" << std::endl;
      }
    } else {
      std::cout << key << " " << value << std::endl;
    }
}

void extractSyncsAndProcesses(
    const std::shared_ptr<tchecker::parsing::system_declaration_t> & original_decl,
    const std::shared_ptr<tchecker::parsing::system_declaration_t> & prop_decl,
    const std::shared_ptr<tchecker::parsing::system_declaration_t> & env_decl,
    std::stringstream & original_syncs,
    std::set<std::string> & prop_processes,
    std::set<std::string> & env_processes)
{

  // extract process names from prop declaration
  for (const auto & decl : prop_decl->declarations()) {
    auto proc =
      std::dynamic_pointer_cast<tchecker::parsing::process_declaration_t const>(decl);

    if (proc != nullptr) {
      prop_processes.insert(proc->name());
    }
  }

  // extract process names from env declaration
  for (const auto & decl : env_decl->declarations()) {
    auto proc =
      std::dynamic_pointer_cast<tchecker::parsing::process_declaration_t const>(decl);

    if (proc != nullptr) {
      env_processes.insert(proc->name());
    }
  }

  // extract synchronization lines from original declaration
  for (const auto & sync : original_decl->get_syncs()) {

    original_syncs << "sync";

    for (const auto & constraint : sync->sync_constraints()) {
      const std::string & proc = constraint->process().name();
      const std::string & event = constraint->event().name();

      original_syncs << ":" << proc << "@" << event;
    }

    original_syncs << "\n";
  }
}

bool check_consistency(
    const tchecker::intrusive_shared_ptr_t<tchecker::make_shared_t<tchecker::graph::reachability::edge_t<
        tchecker::tck_reach::zg_history_aware::node_t, tchecker::tck_reach::zg_history_aware::edge_t>>> & incoming_edge,
    const tchecker::graph::reachability::node_sptr_t<tchecker::tck_reach::zg_history_aware::node_t,
                                                     tchecker::tck_reach::zg_history_aware::edge_t> & src_node,
    const std::size_t number_of_clocks,
    std::string & resetv_string)
{
  const tchecker::clock_constraint_container_t clk_guard_vector = incoming_edge->get_guard_cont();
  for (auto constraint : clk_guard_vector) {
    tchecker::variable_id_t variable_id = (constraint.id1() != tchecker::REFCLOCK_ID) ? constraint.id1() : constraint.id2();
    if (variable_id > resetv_string.size() || resetv_string[variable_id] != '1') {
      return false;
    }
  }
  return true;
}

std::tuple<bool, unsigned long long int, unsigned long long int, unsigned long long int>
backward_propagation(const std::shared_ptr<tchecker::tck_reach::zg_history_aware::graph_t> & graph,
                     std::queue<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> & pi_nodes,
                     std::unordered_set<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> & reachable_waiting_list,
                     std::unordered_set<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> & reachable_visited_list,
                     std::unordered_map<std::string, std::string> & vedge_event_cache,
                     std::set<std::string> & declared_events,
                     std::ostream & file,
                     unsigned long long int & location_counter,
                     std::unordered_map<std::string, unsigned long long int> & node_id_map,
                     std::unordered_map<std::string, bool> & edge_presence_map)
{

  const std::size_t number_of_clocks = graph->zg().system().as_system_system().clocks_count(tchecker::VK_FLATTENED);
  unsigned long long int new_count = 0;
  unsigned long long int visited_states = 0;
  unsigned long long int visited_transition = 0;
  const tchecker::system::system_t graph_system = graph->zg().system().as_system_system();

  while (!pi_nodes.empty()) {

    tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t pi_node = pi_nodes.front();
    auto pi_vloc = tchecker::to_string(pi_node->state_ptr()->vloc(), graph_system);

    visited_states += 1;

    pi_nodes.pop();
    pi_node->update_reach_status(true);
    reachable_visited_list.insert(pi_node);

    std::vector<tchecker::intrusive_shared_ptr_t<tchecker::make_shared_t<tchecker::graph::reachability::edge_t<
        tchecker::tck_reach::zg_history_aware::node_t, tchecker::tck_reach::zg_history_aware::edge_t>>>> edges_copy;

    for (auto e : graph->incoming_edges(pi_node)) {
      edges_copy.push_back(e);
    }
    for (auto incoming_edge : edges_copy) {

      if (graph->edge_src(incoming_edge).ptr() == nullptr || graph->edge_tgt(incoming_edge).ptr() == nullptr) {
        continue;
      }

      const auto edge = graph_system.edge(*incoming_edge->vedge().begin());
      const auto & event_name = graph_system.event_name(edge->event_id());
      auto src_node = graph->edge_src(incoming_edge);

      auto src_vloc = tchecker::to_string(src_node->state_ptr()->vloc(), graph_system);
      std::string resetv_string;
      to_string(src_node->reset_history_vector(), resetv_string);
      std::reverse(resetv_string.begin(), resetv_string.end());

      if (event_name[0] == '_') {

        const bool is_consistent = check_consistency(incoming_edge, src_node, number_of_clocks, resetv_string);
        if (is_consistent == true) {
          reachable_waiting_list.erase(src_node);
          new_count++;
          src_node->final(true);
          pi_nodes.push(src_node);
          graph->remove_outgoing_edges(src_node);

          if (src_node->initial()) {
            return std::make_tuple(true, new_count, visited_states, visited_transition);
          }
        }
        else {
          if (reachable_visited_list.insert(src_node).second) {
            reachable_waiting_list.insert(src_node);

            if (node_id_map.find(src_vloc + resetv_string) == node_id_map.end() && !src_node->initial()) { // if not found in the output graph and not initial
              // add this node to the output graph
              location_counter++;
              node_id_map [src_vloc + resetv_string] = location_counter;
              file << "location:sys:S" << location_counter << "{";
              std::map<std::string, std::string> node_attr;
              graph->attributes(src_node, node_attr);
              tchecker::tck_reach::writeNodeAttributes(file, node_attr, src_node, graph_system);
              file << "}";

              file << "  #";
              for (auto loc_id : src_node->state_ptr()->vloc())
                file << graph_system.location(loc_id)->name() << ",";
              file << "\n";

            } else if (node_id_map.find(std::to_string(hash_value(src_node))) == node_id_map.end() && src_node->initial()) {
              // add this node to the output graph
              location_counter++;
              node_id_map [std::to_string(hash_value(src_node))] = location_counter;
              file << "location:sys:S" << location_counter << "{";
              std::map<std::string, std::string> node_attr;
              graph->attributes(src_node, node_attr);
              tchecker::tck_reach::writeNodeAttributes(file, node_attr, src_node, graph_system);
              file << "}";

              file << "  #";
              for (auto loc_id : src_node->state_ptr()->vloc())
                file << graph_system.location(loc_id)->name() << ",";
              file << "\n";
            }

            visited_transition += 1;

            // add this edge to the output graph
            std::map<std::string, std::string> edge_attr;
            graph->attributes(incoming_edge, edge_attr);
            if (src_node->initial()) {
              tchecker::tck_reach::writeEdge(file, node_id_map [std::to_string(hash_value(src_node))], node_id_map ["Pi"], incoming_edge, edge_attr, declared_events, graph_system, vedge_event_cache, edge_presence_map);
            } else {
              tchecker::tck_reach::writeEdge(file, node_id_map [src_vloc + resetv_string], node_id_map ["Pi"], incoming_edge, edge_attr, declared_events, graph_system, vedge_event_cache, edge_presence_map);
            }

          }
        }
      }
      else {
        if (reachable_visited_list.insert(src_node).second) {
          reachable_waiting_list.insert(src_node);

          if (node_id_map.find(src_vloc + resetv_string) == node_id_map.end() && !src_node->initial()) { // if not found in the output graph and not initial
            // add this node to the output graph
            location_counter++;
            node_id_map [src_vloc + resetv_string] = location_counter;
            file << "location:sys:S" << location_counter << "{";
            std::map<std::string, std::string> node_attr;
            graph->attributes(src_node, node_attr);
            tchecker::tck_reach::writeNodeAttributes(file, node_attr, src_node, graph_system);
            file << "}";

            file << "  #";
            for (auto loc_id : src_node->state_ptr()->vloc())
              file << graph_system.location(loc_id)->name() << ",";
            file << "\n";

          } else if (node_id_map.find(std::to_string(hash_value(src_node))) == node_id_map.end() && src_node->initial()) {
            // add this node to the output graph
            location_counter++;
            node_id_map [std::to_string(hash_value(src_node))] = location_counter;
            file << "location:sys:S" << location_counter << "{";
            std::map<std::string, std::string> node_attr;
            graph->attributes(src_node, node_attr);
            tchecker::tck_reach::writeNodeAttributes(file, node_attr, src_node, graph_system);
            file << "}";

            file << "  #";
            for (auto loc_id : src_node->state_ptr()->vloc())
              file << graph_system.location(loc_id)->name() << ",";
            file << "\n";
          }

          visited_transition += 1;

          // add this edge to the output graph
          std::map<std::string, std::string> edge_attr;
          graph->attributes(incoming_edge, edge_attr);
          if (src_node->initial()) {
            tchecker::tck_reach::writeEdge(file, node_id_map [std::to_string(hash_value(src_node))], node_id_map ["Pi"], incoming_edge, edge_attr, declared_events, graph_system, vedge_event_cache, edge_presence_map);
          } else {
            tchecker::tck_reach::writeEdge(file, node_id_map [src_vloc + resetv_string], node_id_map ["Pi"], incoming_edge, edge_attr, declared_events, graph_system, vedge_event_cache, edge_presence_map);
          }

        }
      }
      src_node->update_reach_status(true);
    }
  }

  return std::make_tuple(false, new_count, visited_states, visited_transition);
}

std::tuple<unsigned long long int, unsigned long long int> backward_reachability(
    const std::shared_ptr<tchecker::tck_reach::zg_history_aware::graph_t> & graph,
    const std::unordered_set<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> & reachable_waiting_list,
    std::unordered_set<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> & reachable_visited_list,
    std::unordered_map<std::string, std::string> & vedge_event_cache,
    std::set<std::string> & declared_events,
    std::ostream & file,
    unsigned long long int & location_counter,
    std::unordered_map<std::string, unsigned long long int> & node_id_map,
    std::unordered_map<std::string, bool> & edge_presence_map)
{
  std::queue<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> second_reachable_waiting_list;

  unsigned long long int visited_states = 0;
  unsigned long long int visited_transition = 0;
  auto graph_system = graph->zg().system().as_system_system();

  for (const auto & node : reachable_waiting_list) {
    visited_states += 1;

    auto node_vloc = tchecker::to_string(node->state_ptr()->vloc(), graph_system);

    auto incoming_edges = graph->incoming_edges(node);
    for (auto incoming_edge : incoming_edges) {

      auto src_node = graph->edge_src(incoming_edge);
      auto src_vloc = tchecker::to_string(src_node->state_ptr()->vloc(), graph_system);
      std::string resetv_string;
      to_string(src_node->reset_history_vector(), resetv_string);
      std::reverse(resetv_string.begin(), resetv_string.end());

      if (node_id_map.find(src_vloc + resetv_string) == node_id_map.end() && !src_node->initial()) { // if not found in the output graph and not initial
        // add this node to the output graph
        location_counter++;
        node_id_map [src_vloc + resetv_string] = location_counter;
        file << "location:sys:S" << location_counter << "{";
        std::map<std::string, std::string> node_attr;
        graph->attributes(src_node, node_attr);
        tchecker::tck_reach::writeNodeAttributes(file, node_attr, src_node, graph_system);
        file << "}";

        file << "  #";
        for (auto loc_id : src_node->state_ptr()->vloc())
          file << graph_system.location(loc_id)->name() << ",";
        file << "\n";

      } else if (node_id_map.find(std::to_string(hash_value(src_node))) == node_id_map.end() && src_node->initial()) {
        // add this node to the output graph
        location_counter++;
        node_id_map [std::to_string(hash_value(src_node))] = location_counter;
        file << "location:sys:S" << location_counter << "{";
        std::map<std::string, std::string> node_attr;
        graph->attributes(src_node, node_attr);
        tchecker::tck_reach::writeNodeAttributes(file, node_attr, src_node, graph_system);
        file << "}";

        file << "  #";
        for (auto loc_id : src_node->state_ptr()->vloc())
          file << graph_system.location(loc_id)->name() << ",";
        file << "\n";
      }

      visited_transition += 1;

      // add this edge to the output graph
      std::map<std::string, std::string> edge_attr;
      graph->attributes(incoming_edge, edge_attr);
      std::string tgt_resetv_string;
      to_string(node->reset_history_vector(), tgt_resetv_string);
      std::reverse(tgt_resetv_string.begin(), tgt_resetv_string.end());
      if (src_node->initial()) {
        if (node->initial()) {
          tchecker::tck_reach::writeEdge(file, node_id_map [std::to_string(hash_value(src_node))], node_id_map [std::to_string(hash_value(node))], incoming_edge, edge_attr, declared_events, graph_system, vedge_event_cache, edge_presence_map);
        } else {
          tchecker::tck_reach::writeEdge(file, node_id_map [std::to_string(hash_value(src_node))], node_id_map [node_vloc + tgt_resetv_string], incoming_edge, edge_attr, declared_events, graph_system, vedge_event_cache, edge_presence_map);
        }
      } else {
        if (node->initial()) {
          tchecker::tck_reach::writeEdge(file, node_id_map [src_vloc + resetv_string], node_id_map [std::to_string(hash_value(node))], incoming_edge, edge_attr, declared_events, graph_system, vedge_event_cache, edge_presence_map);
        } else {
          tchecker::tck_reach::writeEdge(file, node_id_map [src_vloc + resetv_string], node_id_map [node_vloc + tgt_resetv_string], incoming_edge, edge_attr, declared_events, graph_system, vedge_event_cache, edge_presence_map);
        }
      }

      if (reachable_visited_list.insert(src_node).second) {
        second_reachable_waiting_list.push(src_node);
        src_node->update_reach_status(true);
      }
    }
  }

  while (!second_reachable_waiting_list.empty()) {

    auto node = second_reachable_waiting_list.front();
    second_reachable_waiting_list.pop();

    visited_states += 1;
    auto node_vloc = tchecker::to_string(node->state_ptr()->vloc(), graph_system);

    auto incoming_edges = graph->incoming_edges(node);
    for (auto incoming_edge : incoming_edges) {

      auto src_node = graph->edge_src(incoming_edge);
      auto src_vloc = tchecker::to_string(src_node->state_ptr()->vloc(), graph_system);
      std::string resetv_string;
      to_string(src_node->reset_history_vector(), resetv_string);
      std::reverse(resetv_string.begin(), resetv_string.end());

      if (node_id_map.find(src_vloc + resetv_string) == node_id_map.end() && !src_node->initial()) { // if not found in the output graph and not initial
        // add this node to the output graph
        location_counter++;
        node_id_map [src_vloc + resetv_string] = location_counter;
        file << "location:sys:S" << location_counter << "{";
        std::map<std::string, std::string> node_attr;
        graph->attributes(src_node, node_attr);
        tchecker::tck_reach::writeNodeAttributes(file, node_attr, src_node, graph_system);
        file << "}";

        file << "  #";
        for (auto loc_id : src_node->state_ptr()->vloc())
          file << graph_system.location(loc_id)->name() << ",";
        file << "\n";

      } else if (node_id_map.find(std::to_string(hash_value(src_node))) == node_id_map.end() && src_node->initial()) {
        // add this node to the output graph
        location_counter++;
        node_id_map [std::to_string(hash_value(src_node))] = location_counter;
        file << "location:sys:S" << location_counter << "{";
        std::map<std::string, std::string> node_attr;
        graph->attributes(src_node, node_attr);
        tchecker::tck_reach::writeNodeAttributes(file, node_attr, src_node, graph_system);
        file << "}";

        file << "  #";
        for (auto loc_id : src_node->state_ptr()->vloc())
          file << graph_system.location(loc_id)->name() << ",";
        file << "\n";
      }

      visited_transition += 1;

      // add this edge to the output graph
      std::map<std::string, std::string> edge_attr;
      graph->attributes(incoming_edge, edge_attr);
      std::string tgt_resetv_string;
      to_string(node->reset_history_vector(), tgt_resetv_string);
      std::reverse(tgt_resetv_string.begin(), tgt_resetv_string.end());
      if (src_node->initial()) {
        if (node->initial()) {
          tchecker::tck_reach::writeEdge(file, node_id_map [std::to_string(hash_value(src_node))], node_id_map [std::to_string(hash_value(node))], incoming_edge, edge_attr, declared_events, graph_system, vedge_event_cache, edge_presence_map);
        } else {
          tchecker::tck_reach::writeEdge(file, node_id_map [std::to_string(hash_value(src_node))], node_id_map [node_vloc+tgt_resetv_string], incoming_edge, edge_attr, declared_events, graph_system, vedge_event_cache, edge_presence_map);
        }
      } else {
        if (node->initial()) {
          tchecker::tck_reach::writeEdge(file, node_id_map [src_vloc + resetv_string], node_id_map [std::to_string(hash_value(node))], incoming_edge, edge_attr, declared_events, graph_system, vedge_event_cache, edge_presence_map);
        } else {
          tchecker::tck_reach::writeEdge(file, node_id_map [src_vloc + resetv_string], node_id_map [node_vloc+tgt_resetv_string], incoming_edge, edge_attr, declared_events, graph_system, vedge_event_cache, edge_presence_map);
        }
      }

      if (reachable_visited_list.insert(src_node).second) {
        second_reachable_waiting_list.push(src_node);
        src_node->update_reach_status(true);
      }
    }
  }

  return std::make_tuple(visited_states, visited_transition);
}

/*!
 \brief Perform compositional reachability analysis
*/
void compos(const std::string & input_file, const std::shared_ptr<tchecker::parsing::system_declaration_t> & sysdecl,
            const std::shared_ptr<tchecker::parsing::system_declaration_t> & propertydecl,
            const std::shared_ptr<tchecker::parsing::system_declaration_t> & envdecl)
{
  tchecker::algorithms::reach::stats_t stats;
  std::shared_ptr<tchecker::tck_reach::zg_history_aware::graph_t> graph;
  std::queue<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> pi_nodes;
  unsigned long long int cumulative_visited_states = 0;
  unsigned long long int cumulative_visited_transition = 0;
  bool early_termination = false;
  long long int iteration_num;
  std::string reachable;

  if (early_enabled) {      // this was intended for "iterative" execution and is not yet fully implemented.
    iteration_num = 1;
  } else {
    iteration_num = -1;
  }

  std::chrono::time_point<std::chrono::steady_clock> pi_start_time = std::chrono::steady_clock::now();

  do {

    early_termination = false;

    std::cout << "The algorithm is now running." << std::endl;

    std::tie(stats, graph) = tchecker::tck_reach::zg_history_aware::run(
        propertydecl, envdecl, pi_nodes, early_termination, labels, search_order, block_size, table_size, iteration_num);

    std::cout << "The assumption has been generated." << std::endl;

    // stats
    std::map<std::string, std::string> m;
    stats.attributes(m);
    for (auto && [key, value] : m) {
      if (key == "VISITED_STATES")
        cumulative_visited_states += std::stoull(value);
      else if (key == "VISITED_TRANSITIONS")
        cumulative_visited_transition += std::stoull(value);
      else if (key == "REACHABLE")
        reachable = value;
    }

    if (pi_nodes.empty()) {
      std::chrono::time_point<std::chrono::steady_clock> pi_end_time = std::chrono::steady_clock::now();
      std::chrono::duration<double> pi_duration = pi_end_time - pi_start_time;

      std::cout << std::endl << "REACHABLE: false (Property SATISFIED)" << std::endl;
      std::cout << "TOTAL_RUNNING_TIME: " << pi_duration.count() << std::endl;
      std::cout << "TOTAL_VISITED_STATES: " << cumulative_visited_states << std::endl;
      std::cout << "TOTAL_VISITED_TRANSITIONS: " << cumulative_visited_transition << std::endl;
      std::cout << "Remark: The property is verified to hold for every environment." << std::endl;
      return;
    }

    std::unordered_set<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> reachable_waiting_list;
    std::unordered_set<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> reachable_visited_list;

    std::cout << "Backward error propagation has been initiated." << std::endl;

    std::ostringstream buffer;

    // initialize the middle declaration
    tchecker::tck_reach::graph_parser_prologue (buffer, input_file);

    std::unordered_map<std::string, std::string> vedge_event_cache;
    std::set<std::string> declared_events;

    std::unordered_map<std::string, unsigned long long int> node_id_map;
    std::unordered_map<std::string, bool> edge_presence_map;
    const tchecker::system::system_t graph_system = graph->zg().system().as_system_system();

    unsigned long long int location_counter = 0;

    // add the Pi node to the graph as a special node
    node_id_map ["Pi"] = -1;                // S0 is the Pi node
    buffer << "location:sys:S" << std::to_string(node_id_map ["Pi"]) << "{";
    std::map<std::string, std::string> node_attr;
    graph->attributes(pi_nodes.front(), node_attr);   // write attributes of the first pi node in the list
    tchecker::tck_reach::writeNodeAttributes(buffer, node_attr, pi_nodes.front(), graph_system);
    buffer << "}";

    buffer << "  #";
    for (auto loc_id : pi_nodes.front()->state_ptr()->vloc())
      buffer << graph_system.location(loc_id)->name() << ",";
    buffer << "\n";

    auto [status, new_count, propagation_visited_states, propagation_visited_transitions] = backward_propagation(graph, pi_nodes, reachable_waiting_list, reachable_visited_list, vedge_event_cache, declared_events, buffer, location_counter, node_id_map, edge_presence_map);

    cumulative_visited_states += propagation_visited_states;
    cumulative_visited_transition += propagation_visited_transitions;

    if (status) {
      std::chrono::time_point<std::chrono::steady_clock> pi_end_time = std::chrono::steady_clock::now();
      std::chrono::duration<double> pi_duration = pi_end_time - pi_start_time;

      std::cout << std::endl << "REACHABLE: true (Property VIOLATED)" << std::endl;
      std::cout << "TOTAL_RUNNING_TIME: " << pi_duration.count() << std::endl;
      std::cout << "TOTAL_VISITED_STATES: " << cumulative_visited_states << std::endl;
      std::cout << "TOTAL_VISITED_TRANSITIONS: " << cumulative_visited_transition << std::endl;
      std::cout << "Remark: The property is violated in all environments." << std::endl;
      return;
    }
    auto [reachability_visited_states, reachability_visited_transitions] = backward_reachability(graph, reachable_waiting_list, reachable_visited_list, vedge_event_cache, declared_events, buffer, location_counter, node_id_map, edge_presence_map);

    cumulative_visited_states += reachability_visited_states;
    cumulative_visited_transition += reachability_visited_transitions;

    // finalize the middle declaration

    std::stringstream original_syncs;
    std::set<std::string> prop_processes;
    std::set<std::string> env_processes;

    extractSyncsAndProcesses(sysdecl,propertydecl, envdecl, original_syncs, prop_processes, env_processes);


    std::cout << "Proceeding to the final composition phase targeting the real environment." << std::endl;

    std::shared_ptr<tchecker::parsing::system_declaration_t> check_decl{
        tchecker::tck_reach::graph_parser_epilogue(buffer, env_file, envdecl, propertydecl, os, graph, declared_events, original_syncs, prop_processes, env_processes)};

    auto && [stats_final, graph_final] =
        tchecker::tck_reach::zg_reach_compos::run(sysdecl, check_decl, labels, search_order);

    // stats
    std::map<std::string, std::string> m_final;
    stats_final.attributes(m_final);
    for (auto && [key, value] : m_final) {
      if (key == "VISITED_STATES")
        cumulative_visited_states += std::stoull(value);
      else if (key == "VISITED_TRANSITIONS")
        cumulative_visited_transition += std::stoull(value);
      else if (key == "REACHABLE")
        reachable = value;
    }

    if (reachable == "true") {
      break;
    }

    // clear Pi nodes
    std::queue<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t>().swap(pi_nodes);

    // set the next iteration number
    iteration_num = -1;

  } while (early_termination); // this was intended for "iterative" execution and is not yet fully implemented

   std::chrono::time_point<std::chrono::steady_clock> pi_end_time = std::chrono::steady_clock::now();
   std::chrono::duration<double> pi_duration = pi_end_time - pi_start_time;

  if (reachable == "true") {
    std::cout << std::endl << "REACHABLE: true (Property VIOLATED)" << std::endl;
  } else {
    std::cout << std::endl << "REACHABLE: false (Property SATISFIED)" << std::endl;
  }
   std::cout << "TOTAL_RUNNING_TIME: " << pi_duration.count() << std::endl;
   std::cout << "TOTAL_VISITED_STATES: " << cumulative_visited_states << std::endl;
   std::cout << "TOTAL_VISITED_TRANSITIONS: " << cumulative_visited_transition << std::endl;
}

/*!
 \brief Main function
*/
int main(int argc, char * argv[])
{
  try {
    int optindex = parse_command_line(argc, argv);

    if (argc - optindex > 1) {
      std::cerr << "Too many input files" << std::endl;
      usage(argv[0]);
      return EXIT_FAILURE;
    }

    if (help) {
      usage(argv[0]);
      return EXIT_SUCCESS;
    }

    std::string input_file = (optindex == argc ? "" : argv[optindex]);

    std::shared_ptr<tchecker::parsing::system_declaration_t> sysdecl{load_system_declaration(input_file)};
    std::shared_ptr<tchecker::parsing::system_declaration_t> propertydecl{load_system_declaration(property_file)};
    std::shared_ptr<tchecker::parsing::system_declaration_t> envdecl{load_system_declaration(env_file)};

    if (tchecker::log_error_count() > 0)
      return EXIT_FAILURE;

    std::shared_ptr<std::ofstream> os_ptr{nullptr};

    if (certificate != CERTIFICATE_NONE && output_file != "") {
      try {
        os_ptr = std::make_shared<std::ofstream>(output_file);
        os = os_ptr.get();
      }
      catch (std::exception & e) {
        std::cerr << tchecker::log_error << e.what() << std::endl;
        return EXIT_FAILURE;
      }
    }

    switch (algorithm) {
    case ALGO_REACH:
      reach(*sysdecl);
      break;
    case ALGO_COMPOS:
      compos(input_file, sysdecl, propertydecl, envdecl);
      break;
    default:
      throw std::runtime_error("No algorithm specified");
    }
  }
  catch (std::exception & e) {
    std::cerr << tchecker::log_error << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
