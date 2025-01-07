/*
 * See files AUTHORS and LICENSE for copyright details.
 */

#include <fstream>
#include <getopt.h>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <string>

#include "tchecker/algorithms/reach/algorithm.hh"
#include "tchecker/algorithms/search_order.hh"
#include "tchecker/parsing/parsing.hh"
#include "tchecker/syncprod/system.hh"
#include "tchecker/utils/log.hh"
#include "zg-reach-compos.hh"
#include "zg-reach.hh"

#include "parse-graph.hh"
#include "tchecker/waiting/factory.hh"
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
                                       {
                                           "property-file",
                                           required_argument,
                                           0,
                                           'P',
                                       },
                                       {"env-file", required_argument, 0, 'E'},
                                       {"merge-flag", no_argument, 0, 'm'},
                                       {0, 0, 0, 0}};

static char const * const options = (char *)"a:C:hl:o:s:P:E:m:";

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
  std::cerr << "   -C type       type of certificate" << std::endl;
  std::cerr << "          none       no certificate (default)" << std::endl;
  std::cerr << "          graph      graph of explored state-space" << std::endl;
  std::cerr << "          symbolic   symbolic run to a state with searched labels if any" << std::endl;
  std::cerr << "          concrete   concrete run to a state with searched labels if any (only for reach currently)"
            << std::endl;
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
      case 'm':
        merge_flag = true;
        break;
      case 'P':
        property_file = optarg;
        break;
      case 'E':
        env_file = optarg;
        break;
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
      case 'o':
        output_file = optarg;
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

/*!
 \brief Perform reachability analysis
 \param sysdecl : system declaration
 \post statistics on reachability analysis of command-line specified labels in
 the system declared by sysdecl have been output to standard output.
 A certification has been output if required.
*/
void reach(std::shared_ptr<tchecker::parsing::system_declaration_t> const & sysdecl)
{
  auto && [stats, graph] = tchecker::tck_reach::zg_reach::run(sysdecl, labels, search_order, block_size, table_size);

  // stats
  std::map<std::string, std::string> m;
  stats.attributes(m);
  for (auto && [key, value] : m)
    std::cout << key << " " << value << std::endl;

  // certificate
   if (certificate == CERTIFICATE_GRAPH)
     tchecker::tck_reach::zg_reach::dot_output(*os, *graph, sysdecl->name());
   else if ((certificate == CERTIFICATE_CONCRETE) && stats.reachable()) {
     std::unique_ptr<tchecker::tck_reach::zg_reach::cex::concrete_cex_t> cex{
         tchecker::tck_reach::zg_reach::cex::concrete_counter_example(*graph)};
     if (cex->empty())
       throw std::runtime_error("Unable to compute a concrete counter example");
     tchecker::tck_reach::zg_reach::cex::dot_output(*os, *cex, sysdecl->name());
   }
   else if ((certificate == CERTIFICATE_SYMBOLIC) && stats.reachable()) {
     std::unique_ptr<tchecker::tck_reach::zg_reach::cex::symbolic_cex_t> cex{
         tchecker::tck_reach::zg_reach::cex::symbolic_counter_example(*graph)};
     if (cex->empty())
       throw std::runtime_error("Unable to compute a symbolic counter example");
     tchecker::tck_reach::zg_reach::cex::dot_output(*os, *cex, sysdecl->name());
   }
}

bool check_consistency(
    const tchecker::intrusive_shared_ptr_t<tchecker::make_shared_t<tchecker::graph::reachability::edge_t<
        tchecker::tck_reach::zg_history_aware::node_t, tchecker::tck_reach::zg_history_aware::edge_t>>> & incoming_edge,
    const tchecker::graph::reachability::node_sptr_t<tchecker::tck_reach::zg_history_aware::node_t,
                                                     tchecker::tck_reach::zg_history_aware::edge_t> & src_node,
    const std::size_t number_of_clocks)
{
  const tchecker::clock_constraint_container_t clk_guard_vector = incoming_edge->get_transition().guard_container();
  const std::vector<unsigned> intvar_guard_vector = incoming_edge->get_transition().intvar_guard_container();
  for (auto constraint : clk_guard_vector) {
    tchecker::variable_id_t variable_id = (constraint.id1() != tchecker::REFCLOCK_ID) ? constraint.id1() : constraint.id2();
    if (variable_id < src_node->reset_history_vector().size() && !src_node->reset_history_vector()[variable_id]) {
      return false;
    }
  }
  for (auto variable_id : intvar_guard_vector) {
    if (variable_id < src_node->reset_history_vector().size() &&
        !src_node->reset_history_vector()[variable_id + number_of_clocks]) {
      return false;
    }
  }
  return true;
}

std::tuple<bool, int>
backward_propagation(const std::shared_ptr<tchecker::tck_reach::zg_history_aware::graph_t> & graph,
                     std::queue<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> & pi_nodes,
                     std::unordered_set<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> & reachable_waiting_list,
                     std::unordered_set<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> & reachable_visited_list)
{

  const std::size_t number_of_clocks = graph->zg().system().as_system_system().clocks_count(tchecker::VK_FLATTENED);
  int new_count = 0;

  const tchecker::system::system_t graph_system = graph->zg().system().as_system_system();
  while (!pi_nodes.empty()) {
    tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t pi_node = pi_nodes.front();
    pi_nodes.pop();
    pi_node->update_reach_status(true);
    graph->remove_outgoing_edges(pi_node);
    reachable_visited_list.insert(pi_node);

    auto incoming_edges = graph->incoming_edges(pi_node);
    for (auto incoming_edge : graph->incoming_edges(pi_node)) {
      const auto edge = graph_system.edge(*incoming_edge->vedge().begin());
      const auto & event_name = graph_system.event_name(edge->event_id());
      auto src_node = graph->edge_src(incoming_edge);
      if (event_name[0] == '_') {
        const bool is_consistent = check_consistency(incoming_edge, src_node, number_of_clocks);
        if (is_consistent) {
          reachable_waiting_list.erase(src_node);
          new_count++;
          src_node->final(true);                  
          pi_nodes.push(src_node);                
          graph->remove_outgoing_edges(src_node); 
          if (graph->incoming_edges(pi_node).empty()) {
            graph->remove_node(pi_node);
          }
          if (src_node->initial()) {
            return std::make_tuple(true, new_count);
          }
        }
        else {
          if (reachable_visited_list.insert(src_node).second) {
            reachable_waiting_list.insert(src_node);
          }
        }
      }
      else {
        if (reachable_visited_list.insert(src_node).second) {
          reachable_waiting_list.insert(src_node);
        }
      }
      src_node->update_reach_status(true);
    }
  }

  return std::make_tuple(false, new_count);
}

void backward_reachability(
    const std::shared_ptr<tchecker::tck_reach::zg_history_aware::graph_t> & graph,
    const std::unordered_set<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> & reachable_waiting_list,
    std::unordered_set<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> & reachable_visited_list)
{
  std::queue<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> second_reachable_waiting_list;

  for (const auto & node : reachable_waiting_list) {
    auto incoming_edges = graph->incoming_edges(node);
    for (auto incoming_edge : incoming_edges) {
      auto src_node = graph->edge_src(incoming_edge);
      if (reachable_visited_list.insert(src_node).second) {
        second_reachable_waiting_list.push(src_node);
        src_node->update_reach_status(true);
      }
    }
  }

  while (!second_reachable_waiting_list.empty()) {
    auto node = second_reachable_waiting_list.front();
    second_reachable_waiting_list.pop();
    auto incoming_edges = graph->incoming_edges(node);
    for (auto incoming_edge : incoming_edges) {
      auto src_node = graph->edge_src(incoming_edge);
      if (reachable_visited_list.insert(src_node).second) {
        second_reachable_waiting_list.push(src_node);
        src_node->update_reach_status(true);
      }
    }
  }
}

void compos(const std::string & input_file, const std::shared_ptr<tchecker::parsing::system_declaration_t> & sysdecl,
            const std::shared_ptr<tchecker::parsing::system_declaration_t> & propertydecl,
            const std::shared_ptr<tchecker::parsing::system_declaration_t> & envdecl)
{
  tchecker::algorithms::reach::stats_t stats;
  std::shared_ptr<tchecker::tck_reach::zg_history_aware::graph_t> graph;
  std::queue<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> pi_nodes;
  long long int visited_states = 0;
  long long int visited_transition = 0;
  long double running_time = 0;
  long long int iteration_num = 1;
  bool early_termination = false;
  std::string reachable;

  pi_nodes.empty();
  std::tie(stats, graph) = tchecker::tck_reach::zg_history_aware::run(
      propertydecl, envdecl, pi_nodes, early_termination, labels, search_order, block_size, table_size, iteration_num);

  // stats
  std::map<std::string, std::string> m;
  stats.attributes(m);
  for (auto && [key, value] : m) {
    if (key == "VISITED_STATES")
      visited_states += std::stoll(value);
    else if (key == "VISITED_TRANSITIONS")
      visited_transition += std::stoll(value);
    else if (key == "RUNNING_TIME_SECONDS")
      running_time += std::stold(value);
    else if (key == "REACHABLE")
      reachable = value;
  }

  // certificate
  if (certificate == CERTIFICATE_GRAPH)
    tchecker::tck_reach::zg_history_aware::dot_output(*os, *graph, propertydecl->name());
  else if ((certificate == CERTIFICATE_CONCRETE) && stats.reachable()) {
    std::unique_ptr<tchecker::tck_reach::zg_history_aware::cex::concrete_cex_t> cex{
        tchecker::tck_reach::zg_history_aware::cex::concrete_counter_example(*graph)};
    if (cex->empty())
      throw std::runtime_error("Unable to compute a concrete counter example");
    tchecker::tck_reach::zg_history_aware::cex::dot_output(*os, *cex, propertydecl->name());
  }
  else if ((certificate == CERTIFICATE_SYMBOLIC) && stats.reachable()) {
    std::unique_ptr<tchecker::tck_reach::zg_history_aware::cex::symbolic_cex_t> cex{
        tchecker::tck_reach::zg_history_aware::cex::symbolic_counter_example(*graph)};
    if (cex->empty())
      throw std::runtime_error("Unable to compute a symbolic counter example");
    tchecker::tck_reach::zg_history_aware::cex::dot_output(*os, *cex, propertydecl->name());
  }

  if (pi_nodes.empty()) {
    std::cout << "REACHABLE: " << "false" << std::endl;
    std::cout << "TOTAL_RUNNING_TIME: " << running_time << std::endl;
    std::cout << "TOTAL_VISITED_STATES: " << visited_states << std::endl;
    std::cout << "TOTAL_VISITED_TRANSITIONS: " << visited_transition << std::endl;
    std::cout << "BACKWARD_DES_STATES: N/A" << std::endl;
    std::cout << "BACKWARD_REACH_STATES N/A" << std::endl;
    return;
  }

  std::chrono::time_point<std::chrono::steady_clock> pi_start_time = std::chrono::steady_clock::now();
  std::unordered_set<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> reachable_waiting_list;
  std::unordered_set<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> reachable_visited_list;

  auto [status, new_count] = backward_propagation(graph, pi_nodes, reachable_waiting_list, reachable_visited_list);

  if (status) {
    std::cout << "REACHABLE: " << status << std::endl;
    std::cout << "TOTAL_RUNNING_TIME: " << running_time << std::endl;
    std::cout << "TOTAL_VISITED_STATES: " << visited_states << std::endl;
    std::cout << "TOTAL_VISITED_TRANSITIONS: " << visited_transition << std::endl;
    std::cout << "BACKWARD_DES_STATES: " << new_count << std::endl;
    std::cout << "BACKWARD_REACH_STATES N/A" << std::endl;
    return;
  }
  backward_reachability(graph, reachable_waiting_list, reachable_visited_list);

  std::chrono::time_point<std::chrono::steady_clock> pi_end_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> pi_duration = pi_end_time - pi_start_time;
  running_time += pi_duration.count();

  uint32_t nodes_count;
  std::shared_ptr<tchecker::parsing::system_declaration_t> check_decl{
      tchecker::tck_reach::graph_parser(input_file, env_file, propertydecl, graph, os, nodes_count)};
  auto && [stats_final, graph_final] =
      tchecker::tck_reach::zg_reach_compos::run(sysdecl, check_decl, labels, search_order, block_size, table_size);

  // stats
  std::map<std::string, std::string> m_final;
  stats_final.attributes(m_final);
  for (auto && [key, value] : m_final) {
    if (key == "VISITED_STATES")
      visited_states += std::stoll(value);
    else if (key == "VISITED_TRANSITIONS")
      visited_transition += std::stoll(value);
    else if (key == "RUNNING_TIME_SECONDS")
      running_time += std::stold(value);
    else if (key == "REACHABLE")
      reachable = value;
  }

  std::cout << "REACHABLE: " << reachable << std::endl;
  std::cout << "TOTAL_RUNNING_TIME: " << running_time << std::endl;
  std::cout << "TOTAL_VISITED_STATES: " << visited_states << std::endl;
  std::cout << "TOTAL_VISITED_TRANSITIONS: " << visited_transition << std::endl;
  std::cout << "BACKWARD_REACH_STATES " << nodes_count << std::endl;
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

    if ((certificate == CERTIFICATE_CONCRETE) && (algorithm != ALGO_REACH)) {
      std::cerr << "Concrete counter-example is only available for algorithm reach" << std::endl;
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
      reach(sysdecl);
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
