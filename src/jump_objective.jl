function JuMP.set_objective(m::InnerBilevelModel, sense::MOI.OptimizationSense,
    f::JuMP.AbstractJuMPScalar)
    set_mylevel_obj_sense(m, sense)
    set_mylevel_obj_function(m, f)
    level_f = replace_variables(f, bilevel_model(m), mylevel_model(m), mylevel_var_list(m), level(m))
    JuMP.set_objective(mylevel_model(m), sense, level_f)
end
JuMP.objective_sense(m::InnerBilevelModel) = mylevel_obj_sense(m)
JuMP.objective_function_type(m::InnerBilevelModel) = typeof(mylevel_obj_function(m))
JuMP.objective_function(m::InnerBilevelModel) = mylevel_obj_function(m)
function JuMP.objective_function(m::InnerBilevelModel, FT::Type)
    mylevel_obj_function(m) isa FT || error("The objective function is not of type $FT")
    mylevel_obj_function(m)
end

function JuMP.relative_gap(bm::BilevelModel)::Float64
    _check_solver(bm)
    return MOI.get(bm.solver, MOI.RelativeGap())
end
function JuMP.relative_gap(bm::UpperModel)::Float64
    return JuMP.relative_gap(bm.m)
end
function JuMP.dual_objective_value(bm::BilevelModel; result::Int=1)::Float64
    _check_solver(bm)
    return MOI.get(bm.solver, MOI.DualObjectiveValue(result))
end
function JuMP.objective_bound(bm::BilevelModel)::Float64
    _check_solver(bm)
    return MOI.get(bm.solver, MOI.ObjectiveBound())
end
function JuMP.objective_bound(bm::UpperModel)::Float64
    return JuMP.objective_bound(bm.m)
end

function JuMP.set_objective(m::BilevelModel, sense::MOI.OptimizationSense,
    f::JuMP.AbstractJuMPScalar)
    bilevel_obj_error()
end
JuMP.objective_sense(m::BilevelModel) = JuMP.objective_sense(m.upper)# bilevel_obj_error()
JuMP.objective_function_type(model::BilevelModel) = bilevel_obj_error()
JuMP.objective_function(model::BilevelModel) = bilevel_obj_error()
function JuMP.objective_function(model::BilevelModel, FT::Type)
    bilevel_obj_error()
end

bilevel_obj_error() = error("There is no objective for BilevelModel use Upper(.) and Lower(.)")

function JuMP.objective_value(model::BilevelModel)
    _check_solver(model)
    return MOI.get(model.solver, MOI.ObjectiveValue())
end
function JuMP.objective_value(model::UpperModel)
    return JuMP.objective_value(model.m)
end
function JuMP.objective_value(model::LowerModel)
    return lower_objective_value(model.m)
end

function lower_objective_value(model::BilevelModel; result::Int = 1)
    f = JuMP.objective_function(Lower(model))
    # Evaluate the lower objective expression
    return JuMP.value(f, v -> JuMP.value(v, result = result))
    # return JuMP.value(v -> inner_ref_to_value(Lower(model), v), f)
end