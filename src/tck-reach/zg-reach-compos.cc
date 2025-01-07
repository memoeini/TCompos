/*
* This file is a part of the TChecker project.
*
* See files AUTHORS and LICENSE for copyright details.
*
 */

#include <ranges>

#include <boost/dynamic_bitset.hpp>

#include "counter_example.hh"
#include "tchecker/algorithms/search_order.hh"
#include "tchecker/system/static_analysis.hh"
#include "tchecker/ta/system.hh"
#include "tchecker/utils/log.hh"
#include "zg-reach-compos.hh"

namespace tchecker {

namespace tck_reach {

namespace zg_reach_compos {

/* node_t */

node_t::node_t(tchecker::zg::state_sptr_t const & s)
    : tchecker::graph::node_zg_state_t(s)
{
}

node_t::node_t(tchecker::zg::const_state_sptr_t const & s)
    : tchecker::graph::node_zg_state_t(s)
{
}

/* super_node_t */

super_node_t::super_node_t(state_sptr_t & nodes, bool initial, bool final)
    : tchecker::graph::node_flags_t(initial, final), _inner_nodes(nodes)
{
}

state_sptr_t const & super_node_t::inner_nodes() const
{
  return _inner_nodes;
}

/* node_hash_t */

std::size_t node_hash_t::operator()(tchecker::tck_reach::zg_reach_compos::super_node_t const & n) const
{
  auto it = n.inner_nodes().begin();
  auto end = n.inner_nodes().end();

  size_t h = tchecker::zg::shared_hash_value((*it).state());
  ++it;

  for (; it != end; ++it)
    boost::hash_combine(h, tchecker::zg::shared_hash_value((*it).state()));



  return h;
}

/* node_equal_to_t */

bool node_equal_to_t::operator()(tchecker::tck_reach::zg_reach_compos::super_node_t const & n1,
                                 tchecker::tck_reach::zg_reach_compos::super_node_t const & n2) const
{
  auto it1 = n1.inner_nodes().begin();
  auto it2 = n2.inner_nodes().begin();
  auto end1 = n1.inner_nodes().end();
  auto end2 = n2.inner_nodes().end();
  bool is_eql;

  for (; true; ++it1, ++it2) {
    if (it1 == end1 || it2 == end2) {
      if (it1 == end1 && it2 == end2)
        return true;
      return false;
    }
    is_eql = tchecker::zg::shared_equal_to((*it1).state(), (*it2).state());
    if (!is_eql)
      return false;
  }
}

/* edge_t */

edge_t::edge_t(tchecker::zg::transition_t const & t) : tchecker::graph::edge_vedge_t(t.vedge_ptr()) {}

/* graph_t */

graph_t::graph_t(std::shared_ptr<tchecker::zg_compos::zg_t> const & zg, std::size_t block_size, std::size_t table_size)
    : tchecker::graph::reachability::graph_t<tchecker::tck_reach::zg_reach_compos::super_node_t, tchecker::tck_reach::zg_reach_compos::edge_t,
                                             tchecker::tck_reach::zg_reach_compos::node_hash_t,
                                             tchecker::tck_reach::zg_reach_compos::node_equal_to_t>(
          block_size, table_size, tchecker::tck_reach::zg_reach_compos::node_hash_t(),
          tchecker::tck_reach::zg_reach_compos::node_equal_to_t()),
      _zg(zg)
{
}

graph_t::~graph_t()
{
  tchecker::graph::reachability::graph_t<tchecker::tck_reach::zg_reach_compos::super_node_t, tchecker::tck_reach::zg_reach_compos::edge_t,
                                         tchecker::tck_reach::zg_reach_compos::node_hash_t,
                                         tchecker::tck_reach::zg_reach_compos::node_equal_to_t>::clear();
}

void graph_t::attributes(tchecker::tck_reach::zg_reach_compos::super_node_t const & n, std::map<std::string, std::string> & m) const
{
  std::ostringstream ss;
  std::map<std::string, std::string> inner_node_attr;
  int ctr = 0;

  for (const tchecker::tck_reach::zg_reach_compos::node_t & inner_node : n.inner_nodes()) {
    inner_node_attr.clear();
    _zg->attributes(inner_node.state_ptr(), inner_node_attr);

    ss.str("");
    ss << "<";
    for (auto it = inner_node_attr.begin(); it != inner_node_attr.end(); ++it) {
      if (it != inner_node_attr.begin())
        ss << ", ";
      auto && [key, value] = *it;
      ss << key << "=\"" << value << "\"";
    }
    ss << ">";

    m["inner node " + std::to_string(ctr)] = ss.str();
    ctr ++;
  }
  tchecker::graph::attributes(static_cast<tchecker::graph::node_flags_t const &>(n), m);
}

void graph_t::attributes(tchecker::tck_reach::zg_reach_compos::edge_t const & e, std::map<std::string, std::string> & m) const
{
  m["vedge"] = tchecker::to_string(e.vedge(), _zg->system().as_system_system());
}

/* dot_output */

/*!
\class node_lexical_less_t
\brief Less-than order on nodes based on lexical ordering
*/
class node_lexical_less_t {
public:
  /*!
   \brief Less-than order on nodes based on lexical ordering
   \param n1 : a node
   \param n2 : a node
   \return true if n1 is less-than n2 w.r.t. lexical ordering over the states in
   the nodes
  */
  bool operator()(tchecker::tck_reach::zg_reach_compos::graph_t::node_sptr_t const & n1,
                  tchecker::tck_reach::zg_reach_compos::graph_t::node_sptr_t const & n2) const
  {
    auto it1 = n1->inner_nodes().begin();
    auto it2 = n2->inner_nodes().begin();
    auto end1 = n1->inner_nodes().end();
    auto end2 = n2->inner_nodes().end();
    int state_cmp = 0;

    for (; true; ++it1, ++it2) {
      if (it1 == end1) {
        state_cmp = (it2 == end2 ? 0 : -1);
        break;
      }
      if (it2 == end2) {
        state_cmp = 1;
        break;
      }
      int cmp_value = tchecker::zg::lexical_cmp(it1->state(), it2->state());
      if (cmp_value < 0) {
        state_cmp = -1;
        break;
      }
      if (cmp_value > 0) {
        state_cmp = 1;
        break;
      }
    }

    if (state_cmp != 0)
      return (state_cmp < 0);
    return (tchecker::graph::lexical_cmp(static_cast<tchecker::graph::node_flags_t const &>(*n1),
                                         static_cast<tchecker::graph::node_flags_t const &>(*n2)) < 0);
  }
};

/*!
\class edge_lexical_less_t
\brief Less-than ordering on edges based on lexical ordering
*/
class edge_lexical_less_t {
public:
  /*!
   \brief Less-than ordering on edges based on lexical ordering
   \param e1 : an edge
   \param e2 : an edge
   \return true if e1 is less-than  e2 w.r.t. the tuple of edges in e1 and e2
  */
  bool operator()(tchecker::tck_reach::zg_reach_compos::graph_t::edge_sptr_t const & e1,
                  tchecker::tck_reach::zg_reach_compos::graph_t::edge_sptr_t const & e2) const
  {
    return tchecker::lexical_cmp(e1->vedge(), e2->vedge()) < 0;
  }
};

std::ostream & dot_output(std::ostream & os, tchecker::tck_reach::zg_reach_compos::graph_t const & g, std::string const & name)
{
  return tchecker::graph::reachability::dot_output<tchecker::tck_reach::zg_reach_compos::graph_t,
                                                   tchecker::tck_reach::zg_reach_compos::node_lexical_less_t,
                                                   tchecker::tck_reach::zg_reach_compos::edge_lexical_less_t>(os, g, name);
}

/* counter example */
namespace cex {

tchecker::tck_reach::zg_reach_compos::cex::symbolic_cex_t * symbolic_counter_example(tchecker::tck_reach::zg_reach_compos::graph_t const & g)
{
}

std::ostream & dot_output(std::ostream & os, tchecker::tck_reach::zg_reach_compos::cex::symbolic_cex_t const & cex,
                          std::string const & name)
{
  return tchecker::zg::path::symbolic::dot_output(os, cex, name);
}

tchecker::tck_reach::zg_reach_compos::cex::concrete_cex_t * concrete_counter_example(tchecker::tck_reach::zg_reach_compos::graph_t const & g)
{
}

std::ostream & dot_output(std::ostream & os, tchecker::tck_reach::zg_reach_compos::cex::concrete_cex_t const & cex,
                          std::string const & name)
{
  return tchecker::zg::path::concrete::dot_output(os, cex, name);
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
bool accepting(tchecker::tck_reach::zg_reach_compos::graph_t::node_sptr_t const & n, tchecker::zg_compos::zg_t & ts, boost::dynamic_bitset<> const & labels)
{
  if (!labels.none()) {
    for (const auto & node : n->inner_nodes()) {
      if (labels.is_subset_of(ts.labels(node.state_ptr())) && ts.is_valid_final(node.state_ptr())) {
        return true;
      }
    }
  }
  return false;
}

tchecker::algorithms::reach::stats_t run(tchecker::zg_compos::zg_t & zg, tchecker::tck_reach::zg_reach_compos::graph_t & graph, boost::dynamic_bitset<> const & labels,
                                         enum tchecker::waiting::policy_t policy)
{
  using node_sptr_t = typename tchecker::tck_reach::zg_reach_compos::graph_t::node_sptr_t;

  std::unique_ptr<tchecker::waiting::waiting_t<node_sptr_t>> waiting{tchecker::waiting::factory<node_sptr_t>(policy)};

  tchecker::algorithms::reach::stats_t stats;

  stats.set_start_time();

  std::vector<typename tchecker::zg_compos::zg_t::sst_t> sst;

  zg.initial(sst);

  for (auto && [status, s, t] : sst) {
    auto && [is_new_node, initial_node] = graph.add_node(state_sptr_t{s}, false, false);
    initial_node->initial(true);
    if (is_new_node)
      waiting->insert(initial_node);
  }

  // iterate over next nodes
  while (!waiting->empty()) {
    node_sptr_t super_node = waiting->first();
    waiting->remove_first();

    ++stats.visited_states();

    if (accepting(super_node, zg, labels)) {
      super_node->final(true);
      stats.reachable() = true;
      break;
    }

    for (const tchecker::tck_reach::zg_reach_compos::node_t & inner_node : super_node->inner_nodes()) {
      sst.clear();
      zg.next(inner_node.state_ptr(), sst);

      for (auto && [status, s, t] : sst) {
        auto && [is_new_node, next_node] = graph.add_node(state_sptr_t{s}, false, false);
        if (is_new_node)
          waiting->insert(next_node);
        graph.add_edge(super_node, next_node, *t);

        ++stats.visited_transitions();
      }
    }
  }

  waiting->clear();

  stats.set_end_time();

  return stats;
}


/* run */

std::tuple<tchecker::algorithms::reach::stats_t, std::shared_ptr<tchecker::tck_reach::zg_reach_compos::graph_t>>
run(std::shared_ptr<tchecker::parsing::system_declaration_t> const & orgdecl,
    std::shared_ptr<tchecker::parsing::system_declaration_t> const & sysdecl, std::string const & labels,
    std::string const & search_order, std::size_t block_size, std::size_t table_size)
{
  std::shared_ptr<tchecker::ta::system_t const> original_system{new tchecker::ta::system_t{*orgdecl}};
  std::shared_ptr<tchecker::ta::system_t const> system{new tchecker::ta::system_t{*sysdecl}};

  if (!tchecker::system::every_process_has_initial_location(system->as_system_system()))
    std::cerr << tchecker::log_warning << "system has no initial state" << std::endl;

  std::shared_ptr<tchecker::zg_compos::zg_t> zg{tchecker::zg_compos::factory(original_system, system, tchecker::ts::SHARING, tchecker::zg::ELAPSED_SEMANTICS,
                                                               tchecker::zg_compos::EXTRA_M_GLOBAL, block_size, table_size)};

  std::shared_ptr<tchecker::tck_reach::zg_reach_compos::graph_t> graph{
      new tchecker::tck_reach::zg_reach_compos::graph_t{zg, block_size, table_size}};

  boost::dynamic_bitset<> accepting_labels = system->as_syncprod_system().labels(labels);

  enum tchecker::waiting::policy_t policy = tchecker::algorithms::waiting_policy(search_order);

  tchecker::algorithms::reach::stats_t stats = run(*zg, *graph, accepting_labels, policy);

  return std::make_tuple(stats, graph);
}

} // namespace zg_reach_compos

} // end of namespace tck_reach

} // end of namespace tchecker