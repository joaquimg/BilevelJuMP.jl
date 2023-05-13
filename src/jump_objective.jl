function JuMP.set_objective_sense(
    m::InnerBilevelModel,
    sense::MOI.OptimizationSense,
)
    return JuMP.set_objective_sense(mylevel_model(m), sense)
end
function JuMP.set_objective(
    m::InnerBilevelModel,
    sense::MOI.OptimizationSense,
    f::JuMP.AbstractJuMPScalar,
)
    level_f =
        replace_variables(f, bilevel_model(m), mylevel_var_list(m), level(m))
    return JuMP.set_objective(mylevel_model(m), sense, level_f)
end
function JuMP.set_objective(
    m::InnerBilevelModel,
    sense::MOI.OptimizationSense,
    f::Real,
)
    return JuMP.set_objective(mylevel_model(m), sense, f)
end
function JuMP.objective_sense(m::InnerBilevelModel)
    return JuMP.objective_sense(mylevel_model(m))
end
function JuMP.objective_function_type(m::InnerBilevelModel)
    tp = JuMP.objective_function_type(mylevel_model(m))
    return _bilevel_type(m, tp)
end
_bilevel_type(::InnerBilevelModel, ::Type{JuMP.VariableRef}) = BilevelVariableRef
function _bilevel_type(
    ::InnerBilevelModel,
    ::Type{JuMP.GenericAffExpr{C,V}},
) where {C,V}
    return JuMP.GenericAffExpr{C,BilevelVariableRef}
end
function _bilevel_type(
    ::InnerBilevelModel,
    ::Type{JuMP.GenericQuadExpr{C,V}},
) where {C,V}
    return JuMP.GenericAffExpr{C,BilevelVariableRef}
end
# JuMP.objective_function(m::InnerBilevelModel) = mylevel_obj_function(m)
function JuMP.objective_function(m::InnerBilevelModel)
    f = JuMP.objective_function(mylevel_model(m))
    return _reverse_replace_variable(f, m)
end
function JuMP.objective_function(m::InnerBilevelModel, FT::Type)
    f = JuMP.objective_function(mylevel_model(m), FT)
    f2 = _reverse_replace_variable(f, m)
    f2 isa FT ||
        error("The objective function is not of type $FT, show $(typeof(f2))")
    return f2
end

function JuMP.relative_gap(bm::BilevelModel)::Float64
    _check_solver(bm)
    return MOI.get(bm.solver, MOI.RelativeGap())
end
function JuMP.relative_gap(bm::UpperModel)::Float64
    return JuMP.relative_gap(bm.m)
end
function JuMP.dual_objective_value(bm::BilevelModel; result::Int = 1)::Float64
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

function JuMP.set_objective(
    m::BilevelModel,
    sense::MOI.OptimizationSense,
    f::JuMP.AbstractJuMPScalar,
)
    return _bilevel_obj_error()
end
JuMP.objective_sense(m::BilevelModel) = JuMP.objective_sense(m.upper)# _bilevel_obj_error()
JuMP.objective_function_type(model::BilevelModel) = _bilevel_obj_error()
JuMP.objective_function(model::BilevelModel) = _bilevel_obj_error()
function JuMP.objective_function(model::BilevelModel, FT::Type)
    return _bilevel_obj_error()
end

function _bilevel_obj_error()
    return error(
        "There is no objective for BilevelModel use Upper(.) and Lower(.)",
    )
end

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

"""
    lower_objective_value(model::BilevelModel; result::Int = 1)

Return the value of the objective function of the lower level problem.
"""
function lower_objective_value(model::BilevelModel; result::Int = 1)
    f = JuMP.objective_function(Lower(model))
    # Evaluate the lower objective expression
    return JuMP.value(f; result = result)
    # return JuMP.value(v -> JuMP.value(v, result = result), f)
    # return JuMP.value(v -> inner_ref_to_value(Lower(model), v), f)
end

function JuMP.set_objective_coefficient(
    ::UpperModel,
    variable::BilevelVariableRef,
    coeff::Real,
)
    level_var = variable.model.var_upper[variable.idx]
    model = JuMP.owner_model(level_var)
    return JuMP.set_objective_coefficient(model, level_var, coeff)
end
function JuMP.set_objective_coefficient(
    ::LowerModel,
    variable::BilevelVariableRef,
    coeff::Real,
)
    level_var = variable.model.var_lower[variable.idx]
    model = JuMP.owner_model(level_var)
    return JuMP.set_objective_coefficient(model, level_var, coeff)
end
