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
    return JuMP.set_attribute(
        bm,
        MOI.RawOptimizerAttribute(name),
        value,
    )
end

function JuMP.set_attribute(
    bm::BilevelModel,
    attr::MOI.AbstractOptimizerAttribute,
    value
)
    _check_solver(bm)
    return MOI.set(bm.solver, attr, value)
end

function JuMP.get_attribute(bm::BilevelModel, name::AbstractString)
    return JuMP.get_attribute(
        bm,
        MOI.RawOptimizerAttribute(name),
    )
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
