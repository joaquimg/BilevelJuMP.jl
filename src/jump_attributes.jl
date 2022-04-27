function JuMP.solve_time(bm::BilevelModel)
    return bm.solve_time
end
function build_time(bm::BilevelModel)
    return bm.build_time
end

function JuMP.set_optimizer_attribute(bm::BilevelModel, name::String, value)
    _check_solver(bm)
    return JuMP.set_optimizer_attribute(bm, MOI.RawOptimizerAttribute(name), value)
end
function JuMP.set_optimizer_attribute(
    bm::BilevelModel, attr::MOI.AbstractOptimizerAttribute, value
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
    return JuMP.get_optimizer_attribute(bm.solver, MOI.RawOptimizerAttribute(name))
end
function JuMP.get_optimizer_attribute(
    bm::BilevelModel, attr::MOI.AbstractOptimizerAttribute
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

function set_copy_names(bm::BilevelModel)
    bm.copy_names = true
    return nothing
end
function unset_copy_names(bm::BilevelModel)
    bm.copy_names = false
    return nothing
end
function get_copy_names(bm::BilevelModel)
    return bm.copy_names
end
function set_pass_start(bm::BilevelModel)
    bm.pass_start = true
    return nothing
end
function unset_pass_start(bm::BilevelModel)
    bm.pass_start = false
    return nothing
end
function get_pass_start(bm::BilevelModel)
    return bm.pass_start
end