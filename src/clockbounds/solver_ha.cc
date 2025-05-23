/*
 * See files AUTHORS and LICENSE for copyright details.
 */

#include <cassert>
#include <cstring>
#include <unordered_set>

#include "tchecker/clockbounds/solver_ha.hh"
#include "tchecker/expression/static_analysis.hh"

namespace tchecker {

namespace clockbounds_ha {

df_solver_t::df_solver_t(tchecker::ta_ha::system_t const & system)
    : _loc_number(system.locations_count()), _clock_number(system.clock_variables().size(tchecker::VK_FLATTENED)),
      _loc_pid(_loc_number, 0),
      _dim(1 + _loc_number * _clock_number), // 1 variable for each clock in each location, plus 1 for dummy clock 0
      _L(nullptr), _U(nullptr), _has_solution(true)
{
  if ((_loc_number > 0) && (_clock_number > 0) && ((_dim < _loc_number) || (_dim < _clock_number)))
    throw std::invalid_argument("invalid number of clocks or locations (overflow)");

  for (tchecker::loc_id_t id = 0; id < _loc_number; ++id)
    _loc_pid[id] = system.location(id)->pid();

  _L = new tchecker::dbm::db_t[_dim * _dim];
  _U = new tchecker::dbm::db_t[_dim * _dim];
  clear();
}

df_solver_t::df_solver_t(tchecker::clockbounds_ha::df_solver_t const & solver)
    : _loc_number(solver._loc_number), _clock_number(solver._clock_number), _loc_pid(solver._loc_pid), _dim(solver._dim),
      _L(nullptr), _U(nullptr), _has_solution(true)
{
  _L = new tchecker::dbm::db_t[_dim * _dim];
  _U = new tchecker::dbm::db_t[_dim * _dim];
  memcpy(_L, solver._L, _dim * _dim * sizeof(*_L));
  memcpy(_U, solver._U, _dim * _dim * sizeof(*_U));
}

df_solver_t::df_solver_t(tchecker::clockbounds_ha::df_solver_t && solver)
    : _loc_number(std::move(solver._loc_number)), _clock_number(std::move(solver._clock_number)),
      _loc_pid(std::move(solver._loc_pid)), _dim(std::move(solver._dim)), _L(std::move(solver._L)), _U(std::move(solver._U)),
      _has_solution(std::move(solver._has_solution))
{
  solver._loc_number = 0;
  solver._clock_number = 0;
  solver._dim = 1;
  solver._L = nullptr;
  solver._U = nullptr;
  solver._has_solution = false;
}

df_solver_t::~df_solver_t()
{
  delete[] _L;
  delete[] _U;
}

tchecker::clockbounds_ha::df_solver_t & df_solver_t::operator=(tchecker::clockbounds_ha::df_solver_t const & solver)
{
  if (this != &solver) {
    _loc_number = solver._loc_number;
    _clock_number = solver._clock_number;
    _loc_pid = solver._loc_pid;
    _dim = solver._dim;

    delete[] _L;
    _L = new tchecker::dbm::db_t[_dim * _dim];
    memcpy(_L, solver._L, _dim * _dim * sizeof(*_L));

    delete[] _U;
    _U = new tchecker::dbm::db_t[_dim * _dim];
    memcpy(_U, solver._U, _dim * _dim * sizeof(*_U));

    _has_solution = solver._has_solution;
  }

  return *this;
}

tchecker::clockbounds_ha::df_solver_t & df_solver_t::operator=(tchecker::clockbounds_ha::df_solver_t && solver)
{
  if (this != &solver) {
    _loc_number = std::move(solver._loc_number);
    _clock_number = std::move(solver._clock_number);
    _loc_pid = std::move(solver._loc_pid);
    _dim = std::move(solver._dim);
    _L = std::move(solver._L);
    _U = std::move(solver._U);
    _has_solution = std::move(solver._has_solution);

    solver._loc_number = 0;
    solver._clock_number = 0;
    solver._dim = 1;
    solver._L = nullptr;
    solver._U = nullptr;
    solver._has_solution = false;
  }

  return *this;
}

tchecker::clock_id_t df_solver_t::clock_number() const { return _clock_number; }

tchecker::clock_id_t df_solver_t::loc_number() const { return _loc_number; }

tchecker::clockbounds::bound_t df_solver_t::L(tchecker::loc_id_t l, tchecker::clock_id_t x) const
{
  assert(l < _loc_number);
  assert(x < _clock_number);
  tchecker::dbm::db_t db = _L[0 * _dim + index(l, x)];
  return (db == tchecker::dbm::LT_INFINITY ? tchecker::clockbounds::NO_BOUND : -tchecker::dbm::value(db));
}

tchecker::clockbounds::bound_t df_solver_t::U(tchecker::loc_id_t l, tchecker::clock_id_t x) const
{
  assert(l < _loc_number);
  assert(x < _clock_number);
  tchecker::dbm::db_t db = _U[0 * _dim + index(l, x)];
  return (db == tchecker::dbm::LT_INFINITY ? tchecker::clockbounds::NO_BOUND : -tchecker::dbm::value(db));
}

bool df_solver_t::has_solution() const { return _has_solution; }

void df_solver_t::clear()
{
  // <inf everywhere except <=0 on diagonal
  for (tchecker::clock_id_t x = 0; x < _dim; ++x) {
    for (tchecker::clock_id_t y = 0; y < _dim; ++y) {
      _L[x * _dim + y] = tchecker::dbm::LT_INFINITY;
      _U[x * _dim + y] = tchecker::dbm::LT_INFINITY;
    }
    _L[x * _dim + x] = tchecker::dbm::LE_ZERO;
    _U[x * _dim + x] = tchecker::dbm::LE_ZERO;
  }

  _has_solution = true;
}

void df_solver_t::add_lower_bound_guard(tchecker::loc_id_t l, tchecker::clock_id_t x, tchecker::integer_t c)
{
  assert(l < _loc_number);
  assert(x < _clock_number);
  // L_{l, x} >= c  (i.e. 0 - L_{l, x} <= -c)
  _has_solution &= (tchecker::dbm::constrain(_L, _dim, 0, index(l, x), tchecker::LE, -c) != tchecker::dbm::EMPTY);
}

void df_solver_t::add_upper_bound_guard(tchecker::loc_id_t l, tchecker::clock_id_t x, tchecker::integer_t c)
{
  assert(l < _loc_number);
  assert(x < _clock_number);
  // U_{l, x} >= c  (i.e. 0 - U_{l ,x} <= -c)
  _has_solution &= (tchecker::dbm::constrain(_U, _dim, 0, index(l, x), tchecker::LE, -c) != tchecker::dbm::EMPTY);
}

void df_solver_t::add_assignment(tchecker::loc_id_t l1, tchecker::loc_id_t l2, tchecker::clock_id_t x, tchecker::clock_id_t y,
                                 tchecker::integer_t c)
{
  assert(l1 < _loc_number);
  assert(l2 < _loc_number);
  assert(x < _clock_number);
  assert(y < _clock_number);
  // Propagation over the edge: L_{l2,x} - L_{l1,y} <= c / U_{l2,x} - U_{l1,xy} <= c
  _has_solution &= (tchecker::dbm::constrain(_L, _dim, index(l2, x), index(l1, y), tchecker::LE, c) != tchecker::dbm::EMPTY);
  _has_solution &= (tchecker::dbm::constrain(_U, _dim, index(l2, x), index(l1, y), tchecker::LE, c) != tchecker::dbm::EMPTY);

  // Propagation across processes: L_{m,x} - L_{l1,y} <= c / U_{m,x} - U_{l1,y} <= c
  // for every location m in another process
  for (tchecker::loc_id_t m = 0; m < _loc_number; ++m)
    if (_loc_pid[m] != _loc_pid[l1]) {
      _has_solution &= (tchecker::dbm::constrain(_L, _dim, index(m, x), index(l1, y), tchecker::LE, c) != tchecker::dbm::EMPTY);
      _has_solution &= (tchecker::dbm::constrain(_U, _dim, index(m, x), index(l1, y), tchecker::LE, c) != tchecker::dbm::EMPTY);
    }
}

void df_solver_t::add_no_assignement(tchecker::loc_id_t l1, tchecker::loc_id_t l2, tchecker::clock_id_t x)
{
  assert(l1 < _loc_number);
  assert(l2 < _loc_number);
  assert(x < _clock_number);
  // L_{l2,x} - L_{l1,x} <= 0
  _has_solution &= (tchecker::dbm::constrain(_L, _dim, index(l2, x), index(l1, x), tchecker::LE, 0) != tchecker::dbm::EMPTY);
  // U_{l2,x} - U_{l1,x} <= 0
  _has_solution &= (tchecker::dbm::constrain(_U, _dim, index(l2, x), index(l1, x), tchecker::LE, 0) != tchecker::dbm::EMPTY);
}

std::size_t df_solver_t::index(tchecker::loc_id_t l, tchecker::clock_id_t x) const
{
  assert(l < _loc_number);
  assert(x < _clock_number);
  return 1 + l * _clock_number + x;
}

/*!
\class df_solver_updater_t
\brief Update solver constraints from expressions and statements
\see tchecker::clockbounds::df_solver_t for constraints generated from expressions
*/
class df_solver_updater_t : public tchecker::typed_expression_visitor_t, public tchecker::typed_statement_visitor_t {
public:
  /*!
  \brief Constructor
  \param src : source location identifier
  \param tgt : target location identifier
  \param solver : a solver
  \note this updates constraints in solver for locations src and tgt
  */
  df_solver_updater_t(tchecker::loc_id_t src, tchecker::loc_id_t tgt,
                      std::shared_ptr<tchecker::clockbounds_ha::df_solver_t> const & solver)
      : _src(src), _tgt(tgt), _solver(solver)
  {
  }

  /*!
  \brief Copy constructor
  \param updater : an updater
  \post this is a copy of updater
  */
  df_solver_updater_t(tchecker::clockbounds_ha::df_solver_updater_t const & updater)
      : _src(updater._src), _tgt(updater._tgt), _solver(updater._solver)
  {
  }

  /*!
  \brief Move constructor
  \param updater : an updater
  \post updater has been moved to this
  */
  df_solver_updater_t(tchecker::clockbounds_ha::df_solver_updater_t && updater)
      : _src(std::move(updater._src)), _tgt(std::move(updater._tgt)), _solver(std::move(updater._solver))
  {
  }

  /*!
  \brief Destructor
  */
  ~df_solver_updater_t() = default;

  /*!
  \brief Assignment operator
  \param updater : an updater
  \post this is a copy of updater
  \return this
  */
  tchecker::clockbounds_ha::df_solver_updater_t & operator=(tchecker::clockbounds_ha::df_solver_updater_t const & updater)
  {
    if (this != &updater) {
      _src = updater._src;
      _tgt = updater._tgt;
      _solver = updater._solver;
    }
    return *this;
  }

  /*!
  \brief Move assignment operator
  \param updater : an updater
  \post updater has been moved to this
  \return this
  */
  tchecker::clockbounds_ha::df_solver_updater_t & operator=(tchecker::clockbounds_ha::df_solver_updater_t && updater)
  {
    if (this != &updater) {
      _src = std::move(updater._src);
      _tgt = std::move(updater._tgt);
      _solver = std::move(updater._solver);
    }
    return *this;
  }

  /*!
  \brief Visitor
  \post left and right operand have been visited if expr is a logical-and expression
  */
  virtual void visit(tchecker::typed_binary_expression_t const & expr)
  {
    if (expr.binary_operator() == tchecker::EXPR_OP_LAND) {
      expr.left_operand().visit(*this);
      expr.right_operand().visit(*this);
    }
  }

  /*!
  \brief Visitor
  \post sub-expression has been visited
  */
  virtual void visit(tchecker::typed_par_expression_t const & expr) { expr.expr().visit(*this); }

  /*!
  \brief Visitor
  \post If expr is a lower-bound guard (x>c, x>=c, x==c), then a constraint on clock lower bounds has been added to
  _solver for every clock x (x could be an array) w.r.t. bound c (using method add_lower_bound_guard). If expr is
  an upper-bound guard (x<c, x<=c, x==c), then a constraint on clock upper bounds has been added to _solver for
  every clock x (x could be an array) w.r.t. bound c (using method add_upper_bound_guard).
  */
  virtual void visit(tchecker::typed_simple_clkconstr_expression_t const & expr)
  {
    tchecker::range_t<tchecker::clock_id_t> clocks = tchecker::extract_lvalue_variable_ids(expr.clock());
    tchecker::integer_t bound = tchecker::const_evaluate(expr.bound(), tchecker::clockbounds::MAX_BOUND);

    if ((expr.binary_operator() == tchecker::EXPR_OP_LT) || (expr.binary_operator() == tchecker::EXPR_OP_LE) ||
        (expr.binary_operator() == tchecker::EXPR_OP_EQ)) {
      for (tchecker::clock_id_t clock = clocks.begin(); clock != clocks.end(); ++clock)
        _solver->add_upper_bound_guard(_src, clock, bound);
    }

    if ((expr.binary_operator() == tchecker::EXPR_OP_GT) || (expr.binary_operator() == tchecker::EXPR_OP_GE) ||
        (expr.binary_operator() == tchecker::EXPR_OP_EQ)) {
      for (tchecker::clock_id_t clock = clocks.begin(); clock != clocks.end(); ++clock)
        _solver->add_lower_bound_guard(_src, clock, bound);
    }
  }

  /*!
  \brief Visitor
  \throw std::runtime_error : as diagonal constraints are not supported by diagonal-free solver
  */
  virtual void visit(tchecker::typed_diagonal_clkconstr_expression_t const & expr)
  {
    // throw std::runtime_error("unsupported diagonal constraints");
  }

  // Other visitors on expressions
  virtual void visit(tchecker::typed_int_expression_t const &) {}
  virtual void visit(tchecker::typed_var_expression_t const &) {}
  virtual void visit(tchecker::typed_bounded_var_expression_t const &) {}
  virtual void visit(tchecker::typed_array_expression_t const &) {}
  virtual void visit(tchecker::typed_unary_expression_t const &) {}
  virtual void visit(tchecker::typed_ite_expression_t const &) {}

  /*!
  \brief Visitor
  \post first and second statements have been visited
  */
  virtual void visit(tchecker::typed_sequence_statement_t const & stmt)
  {
    stmt.first().visit(*this);
    stmt.second().visit(*this);
  }

  /*!
  \brief Visitor
  \post condition expression, then_stmt, else_stmt are visited
  */
  virtual void visit(tchecker::typed_if_statement_t const & stmt)
  {
    stmt.condition().visit(*this);
    stmt.then_stmt().visit(*this);
    stmt.else_stmt().visit(*this);
  }

  /*!
  \brief Visitor
  \post condition expression and stmt statement are visited
  */
  virtual void visit(tchecker::typed_while_statement_t const & stmt)
  {
    stmt.condition().visit(*this);
    stmt.statement().visit(*this);
  }

  /*!
  \brief Visitor
  \post No constraint generated for clock assignment x:=c
  */
  virtual void visit(tchecker::typed_int_to_clock_assign_statement_t const &) {}

  /*!
  \brief Visitor
  \post For assignment x:=y, a constraint for every clock x (x could be an array) and every clock y (y could be
  an array) has been added to _solver using method add_assignment
  */
  virtual void visit(tchecker::typed_clock_to_clock_assign_statement_t const & stmt)
  {
    tchecker::range_t<tchecker::clock_id_t> lclocks = tchecker::extract_lvalue_variable_ids(stmt.lclock());
    tchecker::range_t<tchecker::clock_id_t> rclocks = tchecker::extract_lvalue_variable_ids(stmt.rclock());

    for (tchecker::clock_id_t lclock = lclocks.begin(); lclock != lclocks.end(); ++lclock)
      for (tchecker::clock_id_t rclock = rclocks.begin(); rclock != rclocks.end(); ++rclock)
        _solver->add_assignment(_src, _tgt, lclock, rclock, 0);
  }

  /*!
  \brief Visitor
  \post For assignment x:=y+c, a constraint for every clock x (x could be an array) and every clock y (y could be
  an array) using value c has been added to _solver using method add_assignment
  */
  virtual void visit(tchecker::typed_sum_to_clock_assign_statement_t const & stmt)
  {
    tchecker::range_t<tchecker::clock_id_t> lclocks = tchecker::extract_lvalue_variable_ids(stmt.lclock());
    tchecker::range_t<tchecker::clock_id_t> rclocks = tchecker::extract_lvalue_variable_ids(stmt.rclock());
    tchecker::integer_t value = tchecker::const_evaluate(stmt.value(), 0); // 0 is worst estimation w.r.t. constraints

    for (tchecker::clock_id_t lclock = lclocks.begin(); lclock != lclocks.end(); ++lclock)
      for (tchecker::clock_id_t rclock = rclocks.begin(); rclock != rclocks.end(); ++rclock)
        _solver->add_assignment(_src, _tgt, lclock, rclock, value);
  }

  // Other visitors on statements
  virtual void visit(tchecker::typed_nop_statement_t const &) {}
  virtual void visit(tchecker::typed_assign_statement_t const &) {}
  virtual void visit(tchecker::typed_local_var_statement_t const & stmt) {}
  virtual void visit(tchecker::typed_local_array_statement_t const & stmt) {}

protected:
  tchecker::loc_id_t _src;                                     /*!< Source location ID */
  tchecker::loc_id_t _tgt;                                     /*!< Target location ID */
  std::shared_ptr<tchecker::clockbounds_ha::df_solver_t> _solver; /*!< Solver */
};

/*!
  \class assigned_clocks_extractor_t
  \brief Computes the set of clock IDs that are *surely* assigned in a statement. While it is possible to determine
  the ID of a clocks that is assigned when the lvalue is a single clock (i.e. x := e), this is not possible when the
  lvalue base variable is an array (i.e. y[e] := f). This class computes the set of clocks that are assigned and that
  can be identified. In the example above, x is such a clock, while y is not, unless expression e is a constant
  expression that can be evaluated statically, then we can identify which cell of array y is assigned
*/
template <class INSERT_ITERATOR> class assigned_clocks_extractor_t : public tchecker::typed_statement_visitor_t {
public:
  /*!
  \brief Constructor
  */
  assigned_clocks_extractor_t(INSERT_ITERATOR & inserter) : _inserter(inserter) {}

  /*!
  \brief Copy constructor
  */
  assigned_clocks_extractor_t(tchecker::clockbounds_ha::assigned_clocks_extractor_t<INSERT_ITERATOR> const &) = default;

  /*!
  \brief Move constructor
  */
  assigned_clocks_extractor_t(tchecker::clockbounds_ha::assigned_clocks_extractor_t<INSERT_ITERATOR> &&) = default;

  /*!
  \brief Destructor
  */
  virtual ~assigned_clocks_extractor_t() = default;

  /*!
  \brief Assignment operator
  */
  tchecker::clockbounds_ha::assigned_clocks_extractor_t<INSERT_ITERATOR> &
  operator=(tchecker::clockbounds_ha::assigned_clocks_extractor_t<INSERT_ITERATOR> const &) = default;

  /*!
  \brief Move assignment operator
  */
  tchecker::clockbounds_ha::assigned_clocks_extractor_t<INSERT_ITERATOR> &
  operator=(tchecker::clockbounds_ha::assigned_clocks_extractor_t<INSERT_ITERATOR> &&) = default;

  /*!
  \brief Visitor
  \post first and second statements have been visited
  */
  virtual void visit(tchecker::typed_sequence_statement_t const & stmt)
  {
    stmt.first().visit(*this);
    stmt.second().visit(*this);
  }

  /*!
  \brief Visitor
  \post then_stmt and else_stmt statements have been visited
  */
  virtual void visit(tchecker::typed_if_statement_t const & stmt)
  {
    stmt.then_stmt().visit(*this);
    stmt.else_stmt().visit(*this);
  }

  /*!
  \brief Visitor
  \post stmt statement has been visited
  */
  virtual void visit(tchecker::typed_while_statement_t const & stmt) { stmt.statement().visit(*this); }

  /*!
  \brief Visitor
  \post the base clock of the lvalue of stmt has been inserted if its ID can be determined
  */
  virtual void visit(tchecker::typed_int_to_clock_assign_statement_t const & stmt)
  {
    tchecker::range_t<tchecker::variable_id_t> range = tchecker::extract_lvalue_variable_ids(stmt.clock());
    if (range.begin() + 1 == range.end())
      _inserter = range.begin();
  }

  /*!
  \brief Visitor
  \post the base clock of the lvalue of stmt has been inserted if its ID can be determined
  */
  virtual void visit(tchecker::typed_clock_to_clock_assign_statement_t const & stmt)
  {
    tchecker::range_t<tchecker::variable_id_t> range = tchecker::extract_lvalue_variable_ids(stmt.lclock());
    if (range.begin() + 1 == range.end())
      _inserter = range.begin();
  }

  /*!
  \brief Visitor
  \post the base clock of the lvalue of stmt has been inserted if its ID can be determined
  */
  virtual void visit(tchecker::typed_sum_to_clock_assign_statement_t const & stmt)
  {
    tchecker::range_t<tchecker::variable_id_t> range = tchecker::extract_lvalue_variable_ids(stmt.lclock());
    if (range.begin() + 1 == range.end())
      _inserter = range.begin();
  }

  // Other visitors
  virtual void visit(tchecker::typed_nop_statement_t const &) {}
  virtual void visit(tchecker::typed_assign_statement_t const &) {}
  virtual void visit(tchecker::typed_local_var_statement_t const & stmt) {}
  virtual void visit(tchecker::typed_local_array_statement_t const & stmt) {}

protected:
  INSERT_ITERATOR & _inserter; /*!< Inserter for assigned clock IDs */
};

/* add location/edge constraints to solver */

void add_location_constraints(tchecker::typed_expression_t const & inv, tchecker::loc_id_t loc,
                              std::shared_ptr<tchecker::clockbounds_ha::df_solver_t> const & solver)
{
  tchecker::clockbounds_ha::df_solver_updater_t updater(loc, loc, solver);
  inv.visit(updater);
}

void add_edge_constraints(tchecker::typed_expression_t const & guard, tchecker::typed_statement_t const & stmt,
                          tchecker::loc_id_t src, tchecker::loc_id_t tgt,
                          std::shared_ptr<tchecker::clockbounds_ha::df_solver_t> const & solver)
{
  // guard and statement
  tchecker::clockbounds_ha::df_solver_updater_t updater(src, tgt, solver);
  guard.visit(updater);
  stmt.visit(updater);

  // unassigned clocks
  std::unordered_set<tchecker::clock_id_t> assigned_clocks;
  auto assigned_clocks_inserter = std::inserter(assigned_clocks, assigned_clocks.begin());
  tchecker::clockbounds_ha::assigned_clocks_extractor_t extractor(assigned_clocks_inserter);
  stmt.visit(extractor);

  tchecker::clock_id_t clock_number = solver->clock_number();
  for (tchecker::clock_id_t clock = 0; clock < clock_number; ++clock)
    if (assigned_clocks.find(clock) == assigned_clocks.end()) // clock is not assigned
      solver->add_no_assignement(src, tgt, clock);
}

/* solver */

std::shared_ptr<tchecker::clockbounds_ha::df_solver_t> solve(tchecker::ta_ha::system_t const & system)
{
  std::shared_ptr<tchecker::clockbounds_ha::df_solver_t> solver(new tchecker::clockbounds_ha::df_solver_t(system));

  for (tchecker::system::loc_const_shared_ptr_t const & loc : system.locations())
    tchecker::clockbounds_ha::add_location_constraints(system.invariant(loc->id()), loc->id(), solver);

  for (tchecker::system::edge_const_shared_ptr_t const & edge : system.edges())
    tchecker::clockbounds_ha::add_edge_constraints(system.guard(edge->id()), system.statement(edge->id()), edge->src(),
                                                edge->tgt(), solver);
  return solver;
}

/* fill clock bounds map */

void fill_global_lu_map(tchecker::clockbounds_ha::df_solver_t const & solver, tchecker::clockbounds::global_lu_map_t & map)
{
  if (solver.clock_number() != map.clock_number())
    throw std::invalid_argument("solver and global LU map have incompatible clock numbers");

  if (!solver.has_solution())
    throw std::invalid_argument("clock bounds solver has no solution");

  tchecker::loc_id_t loc_nb = solver.loc_number();
  tchecker::clock_id_t clock_nb = solver.clock_number();

  for (tchecker::loc_id_t loc = 0; loc < loc_nb; ++loc)
    for (tchecker::clock_id_t clock = 0; clock < clock_nb; ++clock) {
      map.L()[clock] = std::max(solver.L(loc, clock), map.L()[clock]);
      map.U()[clock] = std::max(solver.U(loc, clock), map.U()[clock]);
    }
}

void fill_local_lu_map(tchecker::clockbounds_ha::df_solver_t const & solver, tchecker::clockbounds::local_lu_map_t & map)
{
  if (solver.clock_number() != map.clock_number())
    throw std::invalid_argument("solver and local LU map have incompatible clock numbers");

  if (solver.loc_number() != map.loc_number())
    throw std::invalid_argument("solver and local LU map have incompatible location numbers");

  if (!solver.has_solution())
    throw std::invalid_argument("clock bounds solver has no solution");

  tchecker::loc_id_t loc_nb = solver.loc_number();
  tchecker::clock_id_t clock_nb = solver.clock_number();

  for (tchecker::loc_id_t loc = 0; loc < loc_nb; ++loc)
    for (tchecker::clock_id_t clock = 0; clock < clock_nb; ++clock) {
      map.L(loc)[clock] = solver.L(loc, clock);
      map.U(loc)[clock] = solver.U(loc, clock);
    }
}

void fill_global_m_map(tchecker::clockbounds_ha::df_solver_t const & solver, tchecker::clockbounds::global_m_map_t & map)
{
  if (solver.clock_number() != map.clock_number())
    throw std::invalid_argument("solver and global M map have incomparible clock numbers");

  if (!solver.has_solution())
    throw std::invalid_argument("clock bounds solver has no solution");

  tchecker::loc_id_t loc_nb = solver.loc_number();
  tchecker::clock_id_t clock_nb = solver.clock_number();

  for (tchecker::loc_id_t loc = 0; loc < loc_nb; ++loc)
    for (tchecker::clock_id_t clock = 0; clock < clock_nb; ++clock)
      map.M()[clock] = std::max(map.M()[clock], std::max(solver.L(loc, clock), solver.U(loc, clock)));
}

void fill_local_m_map(tchecker::clockbounds_ha::df_solver_t const & solver, tchecker::clockbounds::local_m_map_t & map)
{
  if (solver.clock_number() != map.clock_number())
    throw std::invalid_argument("solver and local M map have incompatible clock numbers");

  if (solver.loc_number() != map.loc_number())
    throw std::invalid_argument("solver and local M map have incompatible location numbers");

  if (!solver.has_solution())
    throw std::invalid_argument("clock bounds solver has no solution");

  tchecker::loc_id_t loc_nb = solver.loc_number();
  tchecker::clock_id_t clock_nb = solver.clock_number();

  for (tchecker::loc_id_t loc = 0; loc < loc_nb; ++loc)
    for (tchecker::clock_id_t clock = 0; clock < clock_nb; ++clock)
      map.M(loc)[clock] = std::max(solver.L(loc, clock), solver.U(loc, clock));
}

bool compute_clockbounds(tchecker::ta_ha::system_t const & system, tchecker::clockbounds::clockbounds_t & clockbounds)
{
  // Solve
  std::shared_ptr<tchecker::clockbounds_ha::df_solver_t> solver = tchecker::clockbounds_ha::solve(system);
  if (!solver->has_solution())
    return false;

  // Fill the maps
  tchecker::clockbounds_ha::fill_global_lu_map(*solver, *clockbounds.global_lu_map());
  tchecker::clockbounds_ha::fill_local_lu_map(*solver, *clockbounds.local_lu_map());
  tchecker::clockbounds_ha::fill_global_m_map(*solver, *clockbounds.global_m_map());
  tchecker::clockbounds_ha::fill_local_m_map(*solver, *clockbounds.local_m_map());
  return true;
}

tchecker::clockbounds::clockbounds_t * compute_clockbounds(tchecker::ta_ha::system_t const & system)
{
  tchecker::clockbounds::clockbounds_t * clockbounds =
      new tchecker::clockbounds::clockbounds_t{static_cast<tchecker::loc_id_t>(system.locations_count()),
                                               static_cast<tchecker::clock_id_t>(system.clocks_count(tchecker::VK_FLATTENED))};

  if (tchecker::clockbounds_ha::compute_clockbounds(system, *clockbounds))
    return clockbounds;

  delete clockbounds;
  return nullptr;
}

} // end of namespace clockbounds

} // end of namespace tchecker
