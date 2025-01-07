/*
 * See files AUTHORS and LICENSE for copyright details.
 */

#ifndef TCHECKER_TCK_SIMULATE_GRAPH_HH
#define TCHECKER_TCK_SIMULATE_GRAPH_HH

/*!
 \file graph.hh
 \brief Simulation graph
*/

#include <cstdlib>
#include <map>
#include <memory>
#include <ostream>
#include <string>

#include "tchecker/graph/edge.hh"
#include "tchecker/graph/node.hh"
#include "tchecker/graph/reachability_graph.hh"
#include "tchecker/syncprod/vedge.hh"
#include "tchecker/utils/shared_objects.hh"
#include "tchecker/zg/zg.hh"

namespace tchecker {

namespace tck_simulate {

/*!
\class node_t
\brief Nodes os simulation graph
*/
class node_t : public tchecker::graph::node_flags_t, public tchecker::graph::node_zg_state_t {
public:
  /*!
   \brief Constructor
   \param s : a zone graph state
   \param initial : initial node flag
   \param final : final node flag
   \post this node keeps a shared pointer to s, and has initial/final node flags as specified
   */
  node_t(tchecker::zg::state_sptr_t const & s, bool initial = false, bool final = false);

  /*!
   \brief Constructor
   \param s : a zone graph state
   \param initial : initial node flag
   \param final : final node flag
   \post this node keeps a shared pointer to s, and has initial/final node flags as specified
   */
  node_t(tchecker::zg::const_state_sptr_t const & s, bool initial = false, bool final = false);
};

/*!
 \class edge_t
 \brief Edge of the simulation graph
*/
class edge_t : public tchecker::graph::edge_vedge_t {
public:
  /*!
   \brief Constructor
   \param t : a zone graph transition
   \post this node keeps a shared pointer on the vedge in t
  */
  edge_t(tchecker::zg::transition_t const & t);
};

/*!
 \class graph_t
 \brief Simulation graph over the zone graph
*/
class graph_t
    : public tchecker::graph::reachability::multigraph_t<tchecker::tck_simulate::node_t, tchecker::tck_simulate::edge_t> {
public:
  /*!
   \brief Constructor
   \param zg : zone graph
   \param block_size : number of objects allocated in a block
   \note this keeps a pointer on zg
  */
  graph_t(std::shared_ptr<tchecker::zg::zg_t> const & zg, std::size_t block_size);

  /*!
   \brief Destructor
  */
  virtual ~graph_t();

  using tchecker::graph::reachability::multigraph_t<tchecker::tck_simulate::node_t, tchecker::tck_simulate::edge_t>::attributes;

protected:
  /*!
   \brief Accessor to node attributes
   \param n : a node
   \param m : a map (key, value) of attributes
   \post attributes of node n have been added to map m
  */
  virtual void attributes(tchecker::tck_simulate::node_t const & n, std::map<std::string, std::string> & m) const;

  /*!
   \brief Accessor to edge attributes
   \param e : an edge
   \param m : a map (key, value) of attributes
   \post attributes of edge e have been added to map m
  */
  virtual void attributes(tchecker::tck_simulate::edge_t const & e, std::map<std::string, std::string> & m) const;

private:
  std::shared_ptr<tchecker::zg::zg_t> _zg; /*!< Zone graph */
};

/*!
 \brief Graph output
 \param os : output stream
 \param g : graph
 \param name : graph name
 \post graph g with name has been output to os
*/
std::ostream & dot_output(std::ostream & os, tchecker::tck_simulate::graph_t const & g, std::string const & name);

} // namespace tck_simulate

} // namespace tchecker

#endif // TCHECKER_TCK_SIMULATE_GRAPH_HH
