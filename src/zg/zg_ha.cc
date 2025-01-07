/*
 * See files AUTHORS and LICENSE for copyright details.
 */

#include <queue>

#include "tchecker/dbm/db.hh"
#include "tchecker/variables/clocks.hh"
#include "tchecker/zg/zg_ha.hh"

namespace tchecker {

namespace zg_ha {

/* Semantics functions */

tchecker::state_status_t initial(tchecker::ta_ha::system_t const & system,
                                 tchecker::intrusive_shared_ptr_t<tchecker::shared_vloc_t> const & vloc,
                                 tchecker::intrusive_shared_ptr_t<tchecker::shared_intval_t> const & intval,
                                 tchecker::intrusive_shared_ptr_t<tchecker::zg::shared_zone_t> const & zone,
                                 tchecker::intrusive_shared_ptr_t<tchecker::shared_vedge_t> const & vedge,
                                 tchecker::clock_constraint_container_t & invariant, tchecker::zg::semantics_t & semantics,
                                 tchecker::zg_ha::extrapolation_t & extrapolation,
                                 tchecker::zg_ha::initial_value_t const & initial_range)
{
  tchecker::state_status_t status = tchecker::ta_ha::initial(system, vloc, intval, vedge, invariant, initial_range);
  if (status != tchecker::STATE_OK)
    return status;

  tchecker::dbm::db_t * dbm = zone->dbm();
  tchecker::clock_id_t dim = zone->dim();
  bool delay_allowed = tchecker::ta_ha::delay_allowed(system, *vloc);

  status = semantics.initial(dbm, dim, delay_allowed, invariant);
  if (status != tchecker::STATE_OK)
    return status;

  extrapolation.extrapolate(dbm, dim, *vloc);

  return tchecker::STATE_OK;
}

tchecker::state_status_t final(tchecker::ta_ha::system_t const & system,
                               tchecker::intrusive_shared_ptr_t<tchecker::shared_vloc_t> const & vloc,
                               tchecker::intrusive_shared_ptr_t<tchecker::shared_intval_t> const & intval,
                               tchecker::intrusive_shared_ptr_t<tchecker::zg::shared_zone_t> const & zone,
                               tchecker::intrusive_shared_ptr_t<tchecker::shared_vedge_t> const & vedge,
                               tchecker::clock_constraint_container_t & invariant, tchecker::zg::semantics_t & semantics,
                               tchecker::zg_ha::extrapolation_t & extrapolation, tchecker::zg_ha::final_value_t const & final_range)
{
  tchecker::state_status_t status = tchecker::ta_ha::final(system, vloc, intval, vedge, invariant, final_range);
  if (status != tchecker::STATE_OK)
    return status;

  tchecker::dbm::db_t * dbm = zone->dbm();
  tchecker::clock_id_t dim = zone->dim();
  bool delay_allowed = tchecker::ta_ha::delay_allowed(system, *vloc);

  status = semantics.final(dbm, dim, delay_allowed, invariant);
  if (status != tchecker::STATE_OK)
    return status;

  extrapolation.extrapolate(dbm, dim, *vloc);

  return tchecker::STATE_OK;
}

tchecker::state_status_t next(tchecker::ta_ha::system_t const & system,
                              tchecker::intrusive_shared_ptr_t<tchecker::shared_vloc_t> const & vloc,
                              tchecker::intrusive_shared_ptr_t<tchecker::shared_intval_t> const & intval,
                              tchecker::intrusive_shared_ptr_t<tchecker::zg::shared_zone_t> const & zone,
                              tchecker::intrusive_shared_ptr_t<tchecker::shared_vedge_t> const & vedge,
                              tchecker::clock_constraint_container_t & src_invariant,
                              tchecker::clock_constraint_container_t & guard,
                              std::vector<unsigned> & intvar_guard,
                              tchecker::clock_reset_container_t & reset,
                              std::vector<std::pair<tchecker::variable_id_t, tchecker::variable_id_t>> & intvar_set,
                              tchecker::clock_constraint_container_t & tgt_invariant, tchecker::zg::semantics_t & semantics,
                              tchecker::zg_ha::extrapolation_t & extrapolation, tchecker::zg_ha::outgoing_edges_value_t const & edges)
{
  bool src_delay_allowed = tchecker::ta_ha::delay_allowed(system, *vloc);

  tchecker::state_status_t status =
      tchecker::ta_ha::next(system, vloc, intval, vedge, src_invariant, guard, intvar_guard, reset, intvar_set, tgt_invariant, edges);
  if (status != tchecker::STATE_OK)
    return status;

  tchecker::dbm::db_t * dbm = zone->dbm();
  tchecker::clock_id_t dim = zone->dim();
  bool tgt_delay_allowed = tchecker::ta_ha::delay_allowed(system, *vloc);

  status = semantics.next(dbm, dim, src_delay_allowed, src_invariant, guard, reset, tgt_delay_allowed, tgt_invariant);
  if (status != tchecker::STATE_OK)
    return status;

  extrapolation.extrapolate(dbm, dim, *vloc);

  return tchecker::STATE_OK;
}

tchecker::state_status_t prev(tchecker::ta_ha::system_t const & system,
                              tchecker::intrusive_shared_ptr_t<tchecker::shared_vloc_t> const & vloc,
                              tchecker::intrusive_shared_ptr_t<tchecker::shared_intval_t> const & intval,
                              tchecker::intrusive_shared_ptr_t<tchecker::zg::shared_zone_t> const & zone,
                              tchecker::intrusive_shared_ptr_t<tchecker::shared_vedge_t> const & vedge,
                              tchecker::clock_constraint_container_t & src_invariant,
                              tchecker::clock_constraint_container_t & guard, std::vector<unsigned> & intvar_guard,
                              tchecker::clock_reset_container_t & reset,
                              std::vector<std::pair<tchecker::variable_id_t, tchecker::variable_id_t>> & intvar_set,
                              tchecker::clock_constraint_container_t & tgt_invariant, tchecker::zg::semantics_t & semantics,
                              tchecker::zg_ha::extrapolation_t & extrapolation, tchecker::zg_ha::incoming_edges_value_t const & edges)
{
  bool tgt_delay_allowed = tchecker::ta_ha::delay_allowed(system, *vloc);

  tchecker::state_status_t status =
      tchecker::ta_ha::prev(system, vloc, intval, vedge, src_invariant, guard, intvar_guard, reset, intvar_set, tgt_invariant, edges);
  if (status != tchecker::STATE_OK)
    return status;

  tchecker::dbm::db_t * dbm = zone->dbm();
  tchecker::clock_id_t dim = zone->dim();
  bool src_delay_allowed = tchecker::ta_ha::delay_allowed(system, *vloc);

  status = semantics.prev(dbm, dim, src_delay_allowed, src_invariant, guard, reset, tgt_delay_allowed, tgt_invariant);
  if (status != tchecker::STATE_OK)
    return status;

  extrapolation.extrapolate(dbm, dim, *vloc);

  return tchecker::STATE_OK;
}

/* labels */

boost::dynamic_bitset<> labels(tchecker::ta_ha::system_t const & system, tchecker::zg::state_t const & s)
{
  return tchecker::ta_ha::labels(system, s);
}

/* is_valid_final */

bool is_valid_final(tchecker::ta_ha::system_t const & system, tchecker::zg::state_t const & s) { return !s.zone().is_empty(); }

/* is_initial */

bool is_initial(tchecker::ta_ha::system_t const & system, tchecker::zg::zone_t const & zone)
{
  assert(zone.dim() == system.clocks_count(tchecker::VK_FLATTENED) + 1);
  return tchecker::dbm::contains_zero(zone.dbm(), zone.dim());
}

bool is_initial(tchecker::ta_ha::system_t const & system, tchecker::zg::state_t const & s)
{
  return tchecker::ta_ha::is_initial(system, s) && tchecker::zg_ha::is_initial(system, s.zone());
}

/* attributes */

void attributes(tchecker::ta_ha::system_t const & system, tchecker::zg::state_t const & s, std::map<std::string, std::string> & m)
{
  tchecker::ta_ha::attributes(system, s, m);
  m["zone"] = tchecker::to_string(s.zone(), system.clock_variables().flattened().index());
}

void attributes(tchecker::ta_ha::system_t const & system, tchecker::zg_ha::transition_t const & t,
                std::map<std::string, std::string> & m)
{
  tchecker::ta_ha::attributes(system, t, m);
}

/* initialize */

tchecker::state_status_t initialize(tchecker::ta_ha::system_t const & system,
                                    tchecker::intrusive_shared_ptr_t<tchecker::shared_vloc_t> const & vloc,
                                    tchecker::intrusive_shared_ptr_t<tchecker::shared_intval_t> const & intval,
                                    tchecker::intrusive_shared_ptr_t<tchecker::zg::shared_zone_t> const & zone,
                                    tchecker::intrusive_shared_ptr_t<tchecker::shared_vedge_t> const & vedge,
                                    tchecker::clock_constraint_container_t & invariant,
                                    std::map<std::string, std::string> const & attributes)
{
  // initialize vloc, intval and vedge from ta
  auto state_status = tchecker::ta_ha::initialize(system, vloc, intval, vedge, invariant, attributes);
  if (state_status != STATE_OK)
    return state_status;

  // initialize zone from attributes["zone"]
  tchecker::clock_constraint_container_t clk_constraints;
  try {
    tchecker::from_string(clk_constraints, system.clock_variables(), attributes.at("zone"));
  }
  catch (...) {
    return tchecker::STATE_BAD;
  }

  tchecker::dbm::db_t * dbm = zone->dbm();
  tchecker::clock_id_t const dim = zone->dim();
  tchecker::dbm::universal_positive(dbm, dim);
  tchecker::dbm::status_t zone_status = tchecker::dbm::constrain(dbm, dim, clk_constraints);
  if (zone_status == tchecker::dbm::EMPTY)
    return tchecker::STATE_BAD;

  // Apply invariant
  zone_status = tchecker::dbm::constrain(dbm, dim, invariant);
  if (zone_status == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_SRC_INVARIANT_VIOLATED;

  return tchecker::STATE_OK;
}

/* zg_t */

zg_t::zg_t(std::shared_ptr<tchecker::ta_ha::system_t const> const & system, enum tchecker::ts::sharing_type_t sharing_type,
           std::shared_ptr<tchecker::zg::semantics_t> const & semantics,
           std::shared_ptr<tchecker::zg_ha::extrapolation_t> const & extrapolation, std::size_t block_size, std::size_t table_size)
    : _system(system), _sharing_type(sharing_type), _semantics(semantics), _extrapolation(extrapolation),
      _state_allocator(block_size, block_size, _system->processes_count(), block_size,
                       _system->intvars_count(tchecker::VK_FLATTENED), block_size,
                       _system->clocks_count(tchecker::VK_FLATTENED) + 1, table_size),
      _transition_allocator(block_size, block_size, _system->processes_count(), table_size)
{
}

initial_range_t zg_t::initial_edges() { return tchecker::zg_ha::initial_edges(*_system); }

void zg_t::initial(tchecker::zg_ha::initial_value_t const & init_edge, std::vector<sst_t> & v, tchecker::state_status_t mask)
{
  tchecker::zg::state_sptr_t s = _state_allocator.construct();
  tchecker::zg_ha::transition_sptr_t t = _transition_allocator.construct();

  tchecker::state_status_t status = tchecker::zg_ha::initial(*_system, s->vloc_ptr(), s->intval_ptr(), s->zone_ptr(), t->vedge_ptr(), t->src_invariant_container(),
                                                          *_semantics, *_extrapolation, init_edge);

  if (status & mask) {
    if (_sharing_type == tchecker::ts::SHARING) {
      share(s);
      share(t);
    }
    v.push_back(std::make_tuple(status, s, t));
  }
}

void zg_t::initial(std::vector<sst_t> & v, tchecker::state_status_t mask) { tchecker::ts::initial(*this, v, mask); }

tchecker::zg_ha::outgoing_edges_range_t zg_t::outgoing_edges(tchecker::zg::const_state_sptr_t const & s)
{
  return tchecker::zg_ha::outgoing_edges(*_system, s->vloc_ptr());
}

void zg_t::next(tchecker::zg::const_state_sptr_t const & s, tchecker::zg_ha::outgoing_edges_value_t const & out_edge,
                std::vector<sst_t> & v, tchecker::state_status_t mask)
{
  tchecker::zg::state_sptr_t nexts = _state_allocator.clone(*s);
  tchecker::zg_ha::transition_sptr_t nextt = _transition_allocator.construct();

  tchecker::state_status_t status = tchecker::zg_ha::next(*_system, nexts->vloc_ptr(), nexts->intval_ptr(), nexts->zone_ptr(), nextt->vedge_ptr(), nextt->src_invariant_container(),
                     nextt->guard_container(), nextt->intvar_guard_container(), nextt->reset_container(), nextt->intvar_set_container(), nextt->tgt_invariant_container(), *_semantics, *_extrapolation, out_edge);

  if (status & mask) {
    if (_sharing_type == tchecker::ts::SHARING) {
      share(nexts);
      share(nextt);
    }
    v.push_back(std::make_tuple(status, nexts, nextt));
  }
}

void zg_t::next(tchecker::zg::const_state_sptr_t const & s, std::vector<sst_t> & v, tchecker::state_status_t mask)
{
  tchecker::ts::next(*this, s, v, mask);
}

// Backward

final_range_t zg_t::final_edges(boost::dynamic_bitset<> const & labels) { return tchecker::zg_ha::final_edges(*_system, labels); }

void zg_t::final(final_value_t const & final_edge, std::vector<sst_t> & v, tchecker::state_status_t mask)
{
  tchecker::zg::state_sptr_t s = _state_allocator.construct();
  tchecker::zg_ha::transition_sptr_t t = _transition_allocator.construct();

  tchecker::state_status_t status = tchecker::zg_ha::final(*_system, s->vloc_ptr(), s->intval_ptr(), s->zone_ptr(), t->vedge_ptr(), t->src_invariant_container(),
                      *_semantics, *_extrapolation, final_edge);

  if (status & mask) {
    if (_sharing_type == tchecker::ts::SHARING) {
      share(s);
      share(t);
    }
    v.push_back(std::make_tuple(status, s, t));
  }
}

void zg_t::final(boost::dynamic_bitset<> const & labels, std::vector<sst_t> & v, tchecker::state_status_t mask)
{
  tchecker::ts::final(*this, labels, v, mask);
}

incoming_edges_range_t zg_t::incoming_edges(tchecker::zg::const_state_sptr_t const & s)
{
  return tchecker::zg_ha::incoming_edges(*_system, s->vloc_ptr());
}

void zg_t::prev(tchecker::zg::const_state_sptr_t const & s, incoming_edges_value_t const & in_edge, std::vector<sst_t> & v,
                tchecker::state_status_t mask)
{
  tchecker::zg::state_sptr_t prevs = _state_allocator.clone(*s);
  tchecker::zg_ha::transition_sptr_t prevt = _transition_allocator.construct();

  tchecker::state_status_t status = tchecker::zg_ha::prev(*_system, prevs->vloc_ptr(), prevs->intval_ptr(), prevs->zone_ptr(), prevt->vedge_ptr(), prevt->src_invariant_container(),
                                                       prevt->guard_container(), prevt->intvar_guard_container(), prevt->reset_container(), prevt->intvar_set_container(),  prevt->tgt_invariant_container(), *_semantics, *_extrapolation, in_edge);

  if (status & mask) {
    if (_sharing_type == tchecker::ts::SHARING) {
      share(prevs);
      share(prevt);
    }
    v.push_back(std::make_tuple(status, prevs, prevt));
  }
}

void zg_t::prev(tchecker::zg::const_state_sptr_t const & s, std::vector<sst_t> & v, tchecker::state_status_t mask)
{
  tchecker::ts::prev(*this, s, v, mask);
}

// Builder

void zg_t::build(std::map<std::string, std::string> const & attributes, std::vector<sst_t> & v, tchecker::state_status_t mask)
{
  tchecker::zg::state_sptr_t s = _state_allocator.construct();
  tchecker::zg_ha::transition_sptr_t t = _transition_allocator.construct();
  tchecker::state_status_t status = tchecker::zg_ha::initialize(*_system, *s, *t, attributes);
  if (status & mask) {
    if (_sharing_type == tchecker::ts::SHARING) {
      share(s);
      share(t);
    }
    v.push_back(std::make_tuple(status, s, t));
  }
}

// Split

void zg_t::split(tchecker::zg::const_state_sptr_t const & s, tchecker::clock_constraint_t const & c,
                 std::vector<tchecker::zg::state_sptr_t> & v)
{
  if (!tchecker::dbm::satisfies(s->zone().dbm(), s->zone().dim(), c))
    v.push_back(clone_and_constrain(s, -c));
  if (!tchecker::dbm::satisfies(s->zone().dbm(), s->zone().dim(), -c))
    v.push_back(clone_and_constrain(s, c));
}

void zg_t::split(tchecker::zg::const_state_sptr_t const & s, tchecker::clock_constraint_container_t const & constraints,
                 std::vector<tchecker::zg::state_sptr_t> & v)
{
  std::vector<tchecker::zg::state_sptr_t> done;
  std::queue<tchecker::zg::state_sptr_t> todo;

  todo.push(_state_allocator.clone(*s));
  for (tchecker::clock_constraint_t const & c : constraints) {
    while (!todo.empty()) {
      split(tchecker::zg::const_state_sptr_t{todo.front()}, c, done);
      todo.pop();
    }
    for (tchecker::zg::state_sptr_t const & split_s : done)
      todo.push(split_s);
    done.clear();
  }

  for (; !todo.empty(); todo.pop())
    v.push_back(todo.front());
}

// Inspector

boost::dynamic_bitset<> zg_t::labels(tchecker::zg::const_state_sptr_t const & s) const
{
  return tchecker::zg_ha::labels(*_system, *s);
}

void zg_t::attributes(tchecker::zg::const_state_sptr_t const & s, std::map<std::string, std::string> & m) const
{
  tchecker::zg_ha::attributes(*_system, *s, m);
}

void zg_t::attributes(tchecker::zg_ha::const_transition_sptr_t const & t, std::map<std::string, std::string> & m) const
{
  tchecker::zg_ha::attributes(*_system, *t, m);
}

bool zg_t::is_valid_final(tchecker::zg::const_state_sptr_t const & s) const
{
  return tchecker::zg_ha::is_valid_final(*_system, *s);
}

bool zg_t::is_initial(tchecker::zg::const_state_sptr_t const & s) const { return tchecker::zg_ha::is_initial(*_system, *s); }

// Sharing

void zg_t::share(tchecker::zg::state_sptr_t & s) { _state_allocator.share(s); }

void zg_t::share(tchecker::zg_ha::transition_sptr_t & t) { _transition_allocator.share(t); }

// Private

tchecker::zg::state_sptr_t zg_t::clone_and_constrain(tchecker::zg::const_state_sptr_t const & s,
                                                     tchecker::clock_constraint_t const & c)
{
  tchecker::zg::state_sptr_t clone_s = _state_allocator.clone(*s);
  tchecker::dbm::constrain(clone_s->zone_ptr()->dbm(), clone_s->zone_ptr()->dim(), c);
  if (!clone_s->zone().is_empty() && _sharing_type == tchecker::ts::SHARING)
    share(clone_s);
  return clone_s;
}

/* tools */

tchecker::zg::state_sptr_t initial(tchecker::zg_ha::zg_t & zg, tchecker::vloc_t const & vloc, tchecker::state_status_t mask)
{
  std::vector<tchecker::zg_ha::zg_t::sst_t> v;
  zg.initial(v, mask);
  for (auto && [status, s, t] : v) {
    if (s->vloc() == vloc)
      return s;
  }
  return nullptr;
}

std::tuple<tchecker::zg::state_sptr_t, tchecker::zg_ha::transition_sptr_t> next(tchecker::zg_ha::zg_t & zg,
                                                                             tchecker::zg::const_state_sptr_t const & s,
                                                                             tchecker::vedge_t const & vedge,
                                                                             tchecker::state_status_t mask)
{
  std::vector<tchecker::zg_ha::zg_t::sst_t> v;
  zg.next(s, v, mask);
  for (auto && [status, nexts, nextt] : v)
    if (nextt->vedge() == vedge)
      return std::make_tuple(nexts, nextt);
  return std::make_tuple(nullptr, nullptr);
}

/* factory */

tchecker::zg_ha::zg_t * factory(std::shared_ptr<tchecker::ta_ha::system_t const> const & system,
                             enum tchecker::ts::sharing_type_t sharing_type, enum tchecker::zg::semantics_type_t semantics_type,
                             enum tchecker::zg_ha::extrapolation_type_t extrapolation_type, std::size_t block_size,
                             std::size_t table_size)
{
  std::shared_ptr<tchecker::zg_ha::extrapolation_t> extrapolation{
      tchecker::zg_ha::extrapolation_factory(extrapolation_type, *system)};
  if (extrapolation.get() == nullptr)
    return nullptr;
  std::shared_ptr<tchecker::zg::semantics_t> semantics{tchecker::zg::semantics_factory(semantics_type)};
  return new tchecker::zg_ha::zg_t(system, sharing_type, semantics, extrapolation, block_size, table_size);
}

tchecker::zg_ha::zg_t * factory(std::shared_ptr<tchecker::ta_ha::system_t const> const & system,
                             enum tchecker::ts::sharing_type_t sharing_type, enum tchecker::zg::semantics_type_t semantics_type,
                             enum tchecker::zg_ha::extrapolation_type_t extrapolation_type,
                             tchecker::clockbounds::clockbounds_t const & clock_bounds, std::size_t block_size,
                             std::size_t table_size)
{
  std::shared_ptr<tchecker::zg_ha::extrapolation_t> extrapolation{
      tchecker::zg_ha::extrapolation_factory(extrapolation_type, clock_bounds)};
  if (extrapolation.get() == nullptr)
    return nullptr;
  std::shared_ptr<tchecker::zg::semantics_t> semantics{tchecker::zg::semantics_factory(semantics_type)};
  return new tchecker::zg_ha::zg_t(system, sharing_type, semantics, extrapolation, block_size, table_size);
}

} // end of namespace zg

} // end of namespace tchecker
