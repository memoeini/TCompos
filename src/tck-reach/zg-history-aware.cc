/*
 * See files AUTHORS and LICENSE for copyright details.
 */

#include <queue>

#include <boost/dynamic_bitset.hpp>

#include "counter_example_ha.hh"
#include "tchecker/algorithms/search_order.hh"
#include "tchecker/system/static_analysis.hh"
#include "tchecker/ta/system_ha.hh"
#include "tchecker/utils/log.hh"
#include "zg-history-aware.hh"

namespace tchecker {

namespace tck_reach {

namespace zg_history_aware {

/* node_t */

node_t::node_t(tchecker::zg::state_sptr_t const & s, bool initial, bool final, boost::dynamic_bitset<> rhv)
    : tchecker::graph::node_flags_t(initial, final), tchecker::graph::node_zg_state_t(s),
      tchecker::graph::node_reset_history(rhv), tchecker::graph::node_reachability(false)
{
}

node_t::node_t(tchecker::zg::const_state_sptr_t const & s, bool initial, bool final, boost::dynamic_bitset<> rhv)
    : tchecker::graph::node_flags_t(initial, final), tchecker::graph::node_zg_state_t(s),
      tchecker::graph::node_reset_history(rhv), tchecker::graph::node_reachability(false)
{
}

/* node_hash_t */

std::size_t node_hash_t::operator()(tchecker::tck_reach::zg_history_aware::node_t const & n) const
{
  return tchecker::zg::shared_hash_value_incl_vector(n.state(), n.reset_history_vector());
}

/* node_equal_to_t */

bool node_equal_to_t::operator()(tchecker::tck_reach::zg_history_aware::node_t const & n1,
                                 tchecker::tck_reach::zg_history_aware::node_t const & n2) const
{
  return tchecker::zg::shared_equal_to_incl_vector(n1.state(), n2.state(), n1.reset_history_vector(),
                                                   n2.reset_history_vector());
}

/* edge_t */

edge_t::edge_t(tchecker::zg_ha::transition_t const & t)
    : tchecker::graph::edge_vedge_t(t.vedge_ptr()), tchecker::graph::edge_rel_transition(t)
{
}

/* graph_t */

graph_t::graph_t(std::shared_ptr<tchecker::zg_ha::zg_t> const & zg, std::size_t block_size, std::size_t table_size)
    : tchecker::graph::reachability::graph_t<
          tchecker::tck_reach::zg_history_aware::node_t, tchecker::tck_reach::zg_history_aware::edge_t,
          tchecker::tck_reach::zg_history_aware::node_hash_t, tchecker::tck_reach::zg_history_aware::node_equal_to_t>(
          block_size, table_size, tchecker::tck_reach::zg_history_aware::node_hash_t(),
          tchecker::tck_reach::zg_history_aware::node_equal_to_t()),
      _zg(zg)
{
}

graph_t::~graph_t()
{
  tchecker::graph::reachability::graph_t<
      tchecker::tck_reach::zg_history_aware::node_t, tchecker::tck_reach::zg_history_aware::edge_t,
      tchecker::tck_reach::zg_history_aware::node_hash_t, tchecker::tck_reach::zg_history_aware::node_equal_to_t>::clear();
}

void graph_t::attributes(tchecker::tck_reach::zg_history_aware::node_t const & n, std::map<std::string, std::string> & m) const
{
  _zg->attributes(n.state_ptr(), m);
  tchecker::graph::attributes(static_cast<tchecker::graph::node_flags_t const &>(n), m);
}

void graph_t::attributes(tchecker::tck_reach::zg_history_aware::edge_t const & e, std::map<std::string, std::string> & m) const
{
  m["vedge"] = tchecker::to_string(e.vedge(), _zg->system().as_system_system());
}

/* dot_output */

std::ostream & dot_output(std::ostream & os, tchecker::tck_reach::zg_history_aware::graph_t const & g, std::string const & name)
{
  return tchecker::graph::reachability::dot_output<tchecker::tck_reach::zg_history_aware::graph_t,
                                                   tchecker::tck_reach::zg_history_aware::node_lexical_less_t,
                                                   tchecker::tck_reach::zg_history_aware::edge_lexical_less_t>(os, g, name);
}

/* counter example */
namespace cex {

tchecker::tck_reach::zg_history_aware::cex::symbolic_cex_t *
symbolic_counter_example(tchecker::tck_reach::zg_history_aware::graph_t const & g)
{
  return tchecker::tck_reach::symbolic_counter_example_zg<tchecker::tck_reach::zg_history_aware::graph_t>(g);
}

std::ostream & dot_output(std::ostream & os, tchecker::tck_reach::zg_history_aware::cex::symbolic_cex_t const & cex,
                          std::string const & name)
{
  return tchecker::zg_ha::path::symbolic::dot_output(os, cex, name);
}

tchecker::tck_reach::zg_history_aware::cex::concrete_cex_t *
concrete_counter_example(tchecker::tck_reach::zg_history_aware::graph_t const & g)
{
  return tchecker::tck_reach::concrete_counter_example_zg<tchecker::tck_reach::zg_history_aware::graph_t>(g);
}

std::ostream & dot_output(std::ostream & os, tchecker::tck_reach::zg_history_aware::cex::concrete_cex_t const & cex,
                          std::string const & name)
{
  return tchecker::zg_ha::path::concrete::dot_output(os, cex, name);
}

} // namespace cex

/*!
 \brief Check if a node is accepting
 \param n : a node
 \param ts : a transition system
 \param labels : a set of labels
 \return true if labels is not empty, and the set of labels in n contain
 labels, and n is a valid final state in ts, false otherwise
 */
bool accepting(tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t const & n, tchecker::zg_ha::zg_t & ts,
               boost::dynamic_bitset<> const & labels)
{
  return !labels.none() && labels.is_subset_of(ts.labels(n->state_ptr())) && ts.is_valid_final(n->state_ptr());
}

tchecker::algorithms::reach::stats_t
run(tchecker::zg_ha::zg_t & zg, std::shared_ptr<tchecker::ta_ha::system_t const> & env,
    tchecker::tck_reach::zg_history_aware::graph_t & graph,
    std::queue<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> & final_nodes_container,
    boost::dynamic_bitset<> const & labels, enum tchecker::waiting::policy_t policy, bool & early_termination, int num_clocks,
    int num_int_vars, long long int iteration_num)
{
  using node_sptr_t = typename tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t;
  auto graph_system = graph.zg().system().as_system_system();

  auto num_clocks_vars = num_clocks + num_int_vars;
  std::unique_ptr<tchecker::waiting::waiting_t<node_sptr_t>> waiting{tchecker::waiting::factory<node_sptr_t>(policy)};

  std::unordered_map<std::string, std::unordered_set<intvar_id_t>> intvars_set_by_env;
  std::vector<std::string> env_intvar_names;
  std::vector<std::string> env_event_names;
  env_intvar_names.reserve(env->integer_variables().index().size());
  env_event_names.reserve(env->edges_count());
  long long int count = 0; // TODO: Wrong results if the variable overflows (which can happen if number of final states surpasses 2^64)

  // get integer variable names
  for (const auto & intvar : env->integer_variables().index()) {
    env_intvar_names.push_back(intvar.second);
  }

  // get event names
  for (const auto & edge : env->edges()) {
    const std::string & event_name = env->event_name(edge->event_id());
    env_event_names.push_back(event_name);
  }

  // Process each edge's attributes and search for "do" attributes
  for (const auto & edge : env->edges()) {
    const std::string & event_name = env->event_name(edge->event_id());
    for (const auto & attr : edge->attributes().range()) {
      if (attr.key() == "do") {
        for (const auto & name : env_intvar_names) {
          if (attr.value().find(name) != std::string::npos) {
            intvars_set_by_env[event_name].insert(graph_system.intvar_id(name));
          }
        }
      }
    }
  }

  tchecker::algorithms::reach::stats_t stats;

  stats.set_start_time();

  std::vector<typename tchecker::zg_ha::zg_t::sst_t> sst;

  typename tchecker::zg_ha::zg_t::initial_range_t init_edges = zg.initial_edges();
  for (typename tchecker::zg_ha::zg_t::initial_value_t && init_edge : init_edges)
    zg.initial(init_edge, sst);

  for (auto && [status, s, t] : sst) {

    boost::dynamic_bitset<> new_reset_clock_history;
    new_reset_clock_history = boost::dynamic_bitset<>(num_clocks_vars); // create a new bit vector
    new_reset_clock_history.set();                                      // set every flag to true (initial vector)
    auto && [is_new_node, initial_node] = graph.add_node(s, false, false, new_reset_clock_history);

    initial_node->initial(true);
    if (is_new_node)
      waiting->insert(initial_node);
  }

  // iterate over next nodes
  sst.clear();
  while (!waiting->empty()) {
    node_sptr_t node = waiting->first();
    waiting->remove_first();

    ++stats.visited_states();

    if (accepting(node, zg, labels)) {
      node->final(true);
      stats.reachable() = true;
      node->update_reach_status(true);
      final_nodes_container.push(node);
      count++;
      if (count == iteration_num) {      // TODO
      early_termination = true;
      break;
      }
    }

    auto src_reset_history = node->reset_history_vector();

    auto node_state = node->state_ptr();
    typename tchecker::zg_ha::zg_t::outgoing_edges_range_t out_edges = zg.outgoing_edges(node_state);
    for (typename tchecker::zg_ha::zg_t::outgoing_edges_value_t && out_edge : out_edges)
      zg.next(node_state, out_edge, sst);

    for (auto && [status, s, t] : sst) {

      auto reset_container = t->reset_container();
      bool is_epsilon = false;

      auto current_edge = graph_system.edge(*t->vedge().begin());
      const auto & current_event_name = graph_system.event_name(current_edge->event_id());
      if (current_event_name[0] == '_') {
        is_epsilon = true;
      }

      boost::dynamic_bitset<> new_reset_clock_history = src_reset_history;
      if (!is_epsilon) { // start from the beginning (all clock flags set to false), then apply the changes
        new_reset_clock_history = boost::dynamic_bitset<>(num_clocks_vars); // create a new bit vector
        for (int i = 0; i < num_clocks_vars; ++i) {                         // set every clock flag to false (initial vector)
          new_reset_clock_history[i] = false;
        }
        for (auto modified_intvar : intvars_set_by_env[current_event_name]) {
          new_reset_clock_history[num_clocks_vars + modified_intvar] = false;
        }
      }

      // apply the changes to clock ids
      for (auto clk_reset_history : t->reset_container()) {
        // set the corresponding flag to true for each resets
        new_reset_clock_history[clk_reset_history.left_id()] = true;
      }

      for (auto intvar_set_history : t->intvar_set_container()) {
        // set the corresponding flag to true for each resets
        new_reset_clock_history[intvar_set_history.first + num_clocks] = true;
      }

      auto && [is_new_node, next_node] = graph.add_node(s, false, false, new_reset_clock_history);

      if (is_new_node)
        waiting->insert(next_node);
      graph.add_edge(node, next_node, *t);

      ++stats.visited_transitions();
    }
    sst.clear();
  }

  waiting->clear();

  stats.set_end_time();

  return stats;
}

/* run */

std::tuple<tchecker::algorithms::reach::stats_t, std::shared_ptr<tchecker::tck_reach::zg_history_aware::graph_t>>
run(std::shared_ptr<tchecker::parsing::system_declaration_t> const & sysdecl,
    std::shared_ptr<tchecker::parsing::system_declaration_t> const & envdecl,
    std::queue<tchecker::tck_reach::zg_history_aware::graph_t::node_sptr_t> & final_nodes_container, bool & early_termination,
    std::string const & labels, std::string const & search_order, std::size_t block_size, std::size_t table_size,
    long long int iteration_num)
{
  std::shared_ptr<tchecker::ta_ha::system_t const> system{new tchecker::ta_ha::system_t{*sysdecl}};
  if (!tchecker::system::every_process_has_initial_location(system->as_system_system()))
    std::cerr << tchecker::log_warning << "system has no initial state" << std::endl;

  std::shared_ptr<tchecker::ta_ha::system_t const> env{new tchecker::ta_ha::system_t{*envdecl}};
  if (!tchecker::system::every_process_has_initial_location(env->as_system_system()))
    std::cerr << tchecker::log_warning << "environment has no initial state" << std::endl;

  std::shared_ptr<tchecker::zg_ha::zg_t> zg{tchecker::zg_ha::factory(
      system, tchecker::ts::SHARING, tchecker::zg::ELAPSED_SEMANTICS, tchecker::zg_ha::EXTRA_M_GLOBAL, block_size, table_size)};

  std::shared_ptr<tchecker::tck_reach::zg_history_aware::graph_t> graph{
      new tchecker::tck_reach::zg_history_aware::graph_t{zg, block_size, table_size}};

  boost::dynamic_bitset<> accepting_labels = system->as_syncprod_system().labels(labels);

  enum tchecker::waiting::policy_t policy = tchecker::algorithms::waiting_policy(search_order);

  int number_of_clocks = system->as_system_system().clocks_count(tchecker::VK_FLATTENED);
  int number_of_int_vars = system->as_system_system().intvars_count(tchecker::VK_FLATTENED);

  tchecker::algorithms::reach::stats_t stats = run(*zg, env, *graph, final_nodes_container, accepting_labels, policy,
                                                   early_termination, number_of_clocks, number_of_int_vars, iteration_num);

  return std::make_tuple(stats, graph);
}

} // namespace zg_history_aware

} // end of namespace tck_reach

} // end of namespace tchecker