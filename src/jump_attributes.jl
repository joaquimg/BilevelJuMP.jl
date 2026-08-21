# Copyright (c) 2019: Joaquim Dias Garcia, and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

function JuMP.solve_time(bm::BilevelModel)
    return bm.solve_time
end

"""
    build_time(model::BilevelModel)

Return the time it took to build the model.
"""
function build_time(bm::BilevelModel)
    return bm.build_time
end

function JuMP.set_attribute(bm::BilevelModel, name::AbstractString, value)
    return JuMP.set_attribute(bm, MOI.RawOptimizerAttribute(name), value)
end

function JuMP.set_attribute(
    bm::BilevelModel,
    attr::MOI.AbstractOptimizerAttribute,
    value,
)
    _check_solver(bm)
    return MOI.set(bm.solver, attr, value)
end

function JuMP.get_attribute(bm::BilevelModel, name::AbstractString)
    return JuMP.get_attribute(bm, MOI.RawOptimizerAttribute(name))
end

function JuMP.get_attribute(
    bm::BilevelModel,
    attr::MOI.AbstractOptimizerAttribute,
)
    _check_solver(bm)
    return MOI.get(bm.solver, attr)
end

function JuMP.set_optimizer_attribute(bm::BilevelModel, name::String, value)
    _check_solver(bm)
    return JuMP.set_optimizer_attribute(
        bm,
        MOI.RawOptimizerAttribute(name),
        value,
    )
end
function JuMP.set_optimizer_attribute(
    bm::BilevelModel,
    attr::MOI.AbstractOptimizerAttribute,
    value,
)
    _check_solver(bm)
    return MOI.set(bm.solver, attr, value)
end
function JuMP.set_optimizer_attributes(bm::BilevelModel, pairs::Pair...)
    for (name, value) in pairs
        JuMP.set_optimizer_attribute(bm.solver, name, value)
    end
end

function JuMP.get_optimizer_attribute(bm::BilevelModel, name::String)
    _check_solver(bm)
    return JuMP.get_optimizer_attribute(
        bm.solver,
        MOI.RawOptimizerAttribute(name),
    )
end
function JuMP.get_optimizer_attribute(
    bm::BilevelModel,
    attr::MOI.AbstractOptimizerAttribute,
)
    _check_solver(bm)
    return MOI.get(bm.solver, attr)
end

function JuMP.set_silent(bm::BilevelModel)
    _check_solver(bm)
    return MOI.set(bm.solver, MOI.Silent(), true)
end
function JuMP.unset_silent(bm::BilevelModel)
    _check_solver(bm)
    return MOI.set(bm.solver, MOI.Silent(), false)
end

function JuMP.set_time_limit_sec(bm::BilevelModel, limit)
    _check_solver(bm)
    return MOI.set(bm.solver, MOI.TimeLimitSec(), limit)
end
function JuMP.unset_time_limit_sec(bm::BilevelModel)
    _check_solver(bm)
    return MOI.set(bm.solver, MOI.TimeLimitSec(), nothing)
end
function JuMP.time_limit_sec(bm::BilevelModel)
    _check_solver(bm)
    return MOI.get(bm.solver, MOI.TimeLimitSec())
end

function JuMP.simplex_iterations(bm::BilevelModel)
    _check_solver(bm)
    return MOI.get(bm.solver, MOI.SimplexIterations())
end

function JuMP.barrier_iterations(bm::BilevelModel)
    _check_solver(bm)
    return MOI.get(bm.solver, MOI.BarrierIterations())
end

function JuMP.node_count(bm::BilevelModel)
    _check_solver(bm)
    return MOI.get(bm.solver, MOI.NodeCount())
end

function JuMP.result_count(bm::BilevelModel)::Int
    _check_solver(bm)
    return MOI.get(bm.solver, MOI.ResultCount())
end

"""
    set_copy_names(model::BilevelModel)

Set the `copy_names` attribute of the solver to `true`.
"""
function set_copy_names(bm::BilevelModel)
    bm.copy_names = true
    return nothing
end

"""
    unset_copy_names(model::BilevelModel)

Set the `copy_names` attribute of the solver to `false`.
"""
function unset_copy_names(bm::BilevelModel)
    bm.copy_names = false
    return nothing
end

"""
    get_copy_names(model::BilevelModel)

Return the value of the `copy_names` attribute of the solver.
"""
function get_copy_names(bm::BilevelModel)
    return bm.copy_names
end

"""
    set_pass_start(model::BilevelModel)

Activate passing start values (both primal and dual) to the solver.
"""
function set_pass_start(bm::BilevelModel)
    bm.pass_start = true
    return nothing
end

"""
    unset_pass_start(model::BilevelModel)

Deactivate passing start values (both primal and dual) to the solver.
"""
function unset_pass_start(bm::BilevelModel)
    bm.pass_start = false
    return nothing
end

"""
    get_pass_start(model::BilevelModel)

Checks if passing start values (both primal and dual) to the solver is activated.
"""
function get_pass_start(bm::BilevelModel)
    return bm.pass_start
end

# Forwarding of solver-specific variable and constraint attributes.
#
# These are only supported for *upper* level variables and constraints. Lower
# level objects are transformed by the reformulation (e.g. lower level
# constraints become variables of the KKT/dual system), so there is no
# one-to-one object in the solver to attach the attribute to.
#
# The solver model only exists after `MOI.copy_to`, which happens inside
# `optimize!`. Hence values are cached on the `BilevelModel` and forwarded by
# `_pass_cached_attributes` right before the solve, so that attributes such as
# `Gurobi.ConstraintAttribute("Lazy")` actually affect it. The cache is kept
# after the solve so that it is replayed on every subsequent `optimize!`.

function _assert_upper(v::BilevelVariableRef)
    if !_in_upper(v)
        error(
            "Setting and getting solver attributes is only supported for " *
            "upper level variables. The variable $(v) belongs to the lower " *
            "level only, which is reformulated, hence it has no direct " *
            "counterpart in the solver.",
        )
    end
    return
end

function _assert_upper(cref::BilevelConstraintRef)
    if !_in_upper(cref)
        error(
            "Setting and getting solver attributes is only supported for " *
            "upper level constraints. The constraint $(cref) belongs to the " *
            "lower level, which is dualized/reformulated, hence it has no " *
            "direct counterpart in the solver.",
        )
    end
    return
end

"""
    _solver_index(v::BilevelVariableRef)

Return the solver `MOI.VariableIndex` of the upper level variable `v`.
Assumes the solver model has already been built.
"""
function _solver_index(v::BilevelVariableRef)
    m = owner_model(v)
    return m.sblm_to_solver[m.upper_to_sblm[JuMP.index(upper_ref(v))]]
end

"""
    _solver_index(cref::BilevelConstraintRef)

Return the solver `MOI.ConstraintIndex` of the upper level constraint `cref`.
Assumes the solver model has already been built.
"""
function _solver_index(cref::BilevelConstraintRef)
    m = owner_model(cref)
    ctr = m.ctr_upper[cref.index]
    return m.sblm_to_solver[m.upper_to_sblm[JuMP.index(ctr)]]
end

_is_built(m::BilevelModel) = m.sblm_to_solver !== nothing

"""
    _pass_cached_attributes(model::BilevelModel)

Forward all cached solver-specific variable and constraint attributes to the
solver. Called during `optimize!`, after the solver model has been built and
before the solve, so that the attributes can affect it.
"""
function _pass_cached_attributes(model::BilevelModel)
    for (idx, attr, value) in model.var_attributes
        vref = BilevelVariableRef(model, idx)
        MOI.set(model.solver, attr, _solver_index(vref), value)
    end
    for (idx, attr, value) in model.ctr_attributes
        cref = BilevelConstraintRef(model, idx)
        MOI.set(model.solver, attr, _solver_index(cref), value)
    end
    return
end

function MOI.set(
    v::BilevelVariableRef,
    attr::MOI.AbstractVariableAttribute,
    value,
)
    m = owner_model(v)
    _check_solver(m)
    _assert_upper(v)
    # keep the last value set for each (variable, attribute) pair
    filter!(t -> !(t[1] == v.idx && t[2] == attr), m.var_attributes)
    push!(m.var_attributes, (v.idx, attr, value))
    if _is_built(m)
        # the solver model already exists, keep it in sync
        MOI.set(m.solver, attr, _solver_index(v), value)
    end
    return
end

function MOI.get(v::BilevelVariableRef, attr::MOI.AbstractVariableAttribute)
    m = owner_model(v)
    _check_solver(m)
    _assert_upper(v)
    if _is_built(m)
        return MOI.get(m.solver, attr, _solver_index(v))
    end
    for (idx, cached, value) in Iterators.reverse(m.var_attributes)
        if idx == v.idx && cached == attr
            return value
        end
    end
    return error(
        "Attribute $(attr) has not been set for the variable $(v), and the " *
        "solver model has not been built yet, so it cannot be queried from " *
        "the solver. Call `optimize!(model)` first.",
    )
end

function MOI.set(
    cref::BilevelConstraintRef,
    attr::MOI.AbstractConstraintAttribute,
    value,
)
    m = owner_model(cref)
    _check_solver(m)
    _assert_upper(cref)
    # keep the last value set for each (constraint, attribute) pair
    filter!(t -> !(t[1] == cref.index && t[2] == attr), m.ctr_attributes)
    push!(m.ctr_attributes, (cref.index, attr, value))
    if _is_built(m)
        # the solver model already exists, keep it in sync
        MOI.set(m.solver, attr, _solver_index(cref), value)
    end
    return
end

function MOI.get(
    cref::BilevelConstraintRef,
    attr::MOI.AbstractConstraintAttribute,
)
    m = owner_model(cref)
    _check_solver(m)
    _assert_upper(cref)
    if _is_built(m)
        return MOI.get(m.solver, attr, _solver_index(cref))
    end
    for (idx, cached, value) in Iterators.reverse(m.ctr_attributes)
        if idx == cref.index && cached == attr
            return value
        end
    end
    return error(
        "Attribute $(attr) has not been set for the constraint $(cref), and " *
        "the solver model has not been built yet, so it cannot be queried " *
        "from the solver. Call `optimize!(model)` first.",
    )
end
