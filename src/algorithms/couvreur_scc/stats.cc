/*
 * See files AUTHORS and LICENSE for copyright details.
 */

#include <sstream>

#include "tchecker/algorithms/couvreur_scc/stats.hh"

namespace tchecker {

namespace algorithms {

namespace couvscc {

stats_t::stats_t() : _visited_states(0), _visited_transitions(0), _stored_states(0), _cycle(false) {}

unsigned long & stats_t::visited_states() { return _visited_states; }

unsigned long stats_t::visited_states() const { return _visited_states; }

unsigned long & stats_t::visited_transitions() { return _visited_transitions; }

unsigned long stats_t::visited_transitions() const { return _visited_transitions; }

unsigned long & stats_t::stored_states() { return _stored_states; }

unsigned long stats_t::stored_states() const { return _stored_states; }

bool & stats_t::cycle() { return _cycle; }

bool stats_t::cycle() const { return _cycle; }

void stats_t::attributes(std::map<std::string, std::string> & m) const
{
  tchecker::algorithms::stats_t::attributes(m);

  std::stringstream sstream;

  sstream.str("");
  sstream << _visited_states;
  m["VISITED_STATES"] = sstream.str();

  sstream.str("");
  sstream << _visited_transitions;
  m["VISITED_TRANSITIONS"] = sstream.str();

  sstream.str("");
  sstream << _stored_states;
  m["STORED_STATES"] = sstream.str();

  sstream.str("");
  sstream << std::boolalpha << _cycle;
  m["CYCLE"] = sstream.str();
}

} // namespace couvscc

} // end of namespace algorithms

} // end of namespace tchecker
