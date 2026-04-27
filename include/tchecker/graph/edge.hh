/*
 * This file is part of the TCompos tool, which builds upon the TChecker framework.
 *
 * See files AUTHORS and LICENSE for copyright details.
 *
 */

#ifndef TCHECKER_GRAPH_EDGE_HH
#define TCHECKER_GRAPH_EDGE_HH

#include <map>
#include <string>

#include "tchecker/syncprod/vedge.hh"
#include "tchecker/zg/zg_ha.hh"

namespace tchecker {

namespace graph {

/*!
\class edge_vedge_t
\brief Edge with a vedge
*/
class edge_vedge_t {
public:
  /*!
   \brief Constructor
   \param vedge : a tuple of edges
   \post this node keeps a shared pointer on vedge
  */
  edge_vedge_t(tchecker::vedge_sptr_t const & vedge);

  /*!
   \brief Constructor
   \param vedge : a tuple of edges
   \post this node keeps a shared pointer on vedge
  */
  edge_vedge_t(tchecker::const_vedge_sptr_t const & vedge);

  /*!
   \brief Accessor
   \return zone graph vedge in this edge
  */
  inline tchecker::vedge_t const & vedge() const { return *_vedge; }

  /*!
   \brief Accessor
   \return shared pointer to the zone graph vedge in this edge
  */
  inline tchecker::intrusive_shared_ptr_t<tchecker::shared_vedge_t const> vedge_ptr() const { return _vedge; }

private:
  tchecker::const_vedge_sptr_t const _vedge; /*!< Tuple of edges */
};

struct edge_rel_transition {

  edge_rel_transition(tchecker::clock_constraint_container_t  guard_cont);

  // inline tchecker::zg::transition_t const & get_transition() const { return _transition; }

  inline tchecker::clock_constraint_container_t const & get_guard_cont() const {return _guard_cont;}

private:
  // zg::transition_t const & _transition;
  tchecker::clock_constraint_container_t _guard_cont;
};

} // namespace graph

} // namespace tchecker

#endif // TCHECKER_GRAPH_EDGE_HH