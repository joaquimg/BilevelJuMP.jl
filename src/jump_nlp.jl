#=
    NLP
    unsafe operations:
    - JuMP._init_NLP
    - JuMP._nlp_objective_function
    These are JuMP non exposed methods (the ones starting with "_")
    Might need to pin JuMP in the future
=#

function _has_nlp_data(model::JuMP.Model)
    return JuMP.nonlinear_model(model) !== nothing
end
function _load_nlp_data(
    model::JuMP.Model,
    _differentiation_backend::MOI.Nonlinear.AbstractAutomaticDifferentiation,
)
    # called in JuMP.optimize!
    # The nlp_data is not kept in sync, so re-set it here.
    # TODO: Consider how to handle incremental solves.
    if JuMP.nonlinear_model(model) !== nothing
        evaluator = MOI.Nonlinear.Evaluator(
            JuMP.nonlinear_model(model),
            _differentiation_backend,
            JuMP.index.(JuMP.all_variables(model)),
        )
        MOI.set(model, MOI.NLPBlock(), MOI.NLPBlockData(evaluator))
    end
    return
end

no_nlp() = error("Nonlinear data must be passed to the Upper(.) model")
function no_nlp_lower()
    return error(
        "Nonlinear data (objective, constraints, parameters) is not allowed in the lower level. " *
        "If you are trying to use the @NLconstraint or @NLobjective macros for quadratic expressions, " *
        "please use the @constraint or @objective macros instead. " *
        "Expressions that are not quadratic nor linear are not supported in the lower level."
    )
end

JuMP._init_NLP(m::UpperModel) = JuMP._init_NLP(mylevel_model(m))
JuMP._init_NLP(m::LowerModel) = no_nlp_lower()

###
### parse
###

function MOI.Nonlinear.parse_expression(
    model::MOI.Nonlinear.Model,
    expr::MOI.Nonlinear.Expression,
    x::BilevelVariableRef,
    parent::Int,
)
    if x.level == LOWER_ONLY
        error(
            "Variable $x is a LOWER_ONLY, and should not belong to upper level nonlinear data.",
        )
    end
    m = Upper(x.model)
    MOI.Nonlinear.parse_expression(
        model,
        expr,
        replace_variables(x, x.model, mylevel_var_list(m), level(m)),
        parent,
    )
    return
end

###
### Nonlinear objectives
###

function JuMP.set_nonlinear_objective(
    model::UpperModel,
    sense::MOI.OptimizationSense,
    x,
)
    return JuMP.set_nonlinear_objective(mylevel_model(model), sense, x)
end
function JuMP.set_nonlinear_objective(
    model::LowerModel,
    sense::MOI.OptimizationSense,
    x,
)
    return no_nlp_lower()
end
function JuMP.set_nonlinear_objective(
    model::BilevelModel,
    sense::MOI.OptimizationSense,
    x,
)
    return no_nlp()
end

function JuMP._nlp_objective_function(model::UpperModel)
    if model.nlp_model === nothing
        return nothing
    end
    return model.nlp_model.objective
end
function JuMP._nlp_objective_function(model::LowerModel)
    return nothing
end

###
### Nonlinear parameters
###

function JuMP.add_nonlinear_parameter(model::UpperModel, value::Real)
    return JuMP.add_nonlinear_parameter(mylevel_model(model), value)
end
function JuMP.add_nonlinear_parameter(model::LowerModel, value::Real)
    return no_nlp_lower()
end
function JuMP.add_nonlinear_parameter(model::BilevelModel, value::Real)
    return no_nlp()
end

###
### Nonlinear expressions
###

function JuMP.add_nonlinear_expression(model::UpperModel, ex)
    return JuMP.add_nonlinear_expression(mylevel_model(model), ex)
end
function JuMP.add_nonlinear_expression(model::LowerModel, ex)
    return no_nlp_lower()
end
function JuMP.add_nonlinear_expression(model::BilevelModel, ex)
    return no_nlp()
end

###
### Nonlinear constraints
###

function JuMP.add_nonlinear_constraint(model::UpperModel, ex::JuMP.Expr)
    return JuMP.add_nonlinear_constraint(mylevel_model(model), ex)
end
function JuMP.add_nonlinear_constraint(model::LowerModel, ex::JuMP.Expr)
    return no_nlp_lower()
end
function JuMP.add_nonlinear_constraint(model::BilevelModel, ex::JuMP.Expr)
    return no_nlp()
end

function JuMP.is_valid(model::UpperModel, c::JuMP.NonlinearConstraintRef)
    return JuMP.is_valid(mylevel_model(model), c)
end
function JuMP.is_valid(model::LowerModel, c::JuMP.NonlinearConstraintRef)
    return false
end
function JuMP.is_valid(model::BilevelModel, c::JuMP.NonlinearConstraintRef)
    return JuMP.is_valid(Upper(model), c)
end

# delition is only from the full bilevel model
function JuMP.delete(model::BilevelModel, c::JuMP.NonlinearConstraintRef)
    return JuMP.delete(model.upper, c)
end
function JuMP.delete(model::InnerBilevelModel, c::JuMP.NonlinearConstraintRef)
    error("For deletion call directly from BilevelModel not its inner models.")
    return
end

function JuMP.num_nonlinear_constraints(model::UpperModel)
    return JuMP.num_nonlinear_constraints(mylevel_model(model))
end
function JuMP.num_nonlinear_constraints(model::LowerModel)
    return 0
end
function JuMP.num_nonlinear_constraints(model::BilevelModel)
    return JuMP.num_nonlinear_constraints(Upper(model))
end

function JuMP.all_nonlinear_constraints(model::UpperModel)
    return JuMP.all_nonlinear_constraints(mylevel_model(model))
end
function JuMP.all_nonlinear_constraints(model::LowerModel)
    return JuMP.NonlinearConstraintRef[]
end
function JuMP.all_nonlinear_constraints(model::BilevelModel)
    return JuMP.all_nonlinear_constraints(Upper(model))
end

function JuMP.nonlinear_dual_start_value(model::UpperModel)
    return JuMP.nonlinear_dual_start_value(mylevel_model(model))
end
function JuMP.nonlinear_dual_start_value(model::LowerModel)
    return Float64[]
end
function JuMP.nonlinear_dual_start_value(model::BilevelModel)
    return error(
        "JuMP.nonlinear_dual_start_value should be called in a inner model.",
    )
end

function JuMP.set_nonlinear_dual_start_value(
    model::UpperModel,
    start::Vector{Float64},
)
    return JuMP.set_nonlinear_dual_start_value(mylevel_model(model), start)
end
function JuMP.set_nonlinear_dual_start_value(
    model::LowerModel,
    start::Vector{Float64},
)
    return no_nlp_lower()
end
function JuMP.set_nonlinear_dual_start_value(
    model::BilevelModel,
    start::Vector{Float64},
)
    return no_nlp()
end

function JuMP.set_nonlinear_dual_start_value(model::UpperModel, start::Nothing)
    return JuMP.set_nonlinear_dual_start_value(mylevel_model(model), start)
end
function JuMP.set_nonlinear_dual_start_value(model::LowerModel, start::Nothing)
    return no_nlp_lower()
end
function JuMP.set_nonlinear_dual_start_value(
    model::BilevelModel,
    start::Nothing,
)
    return no_nlp()
end

function JuMP.register(model::UpperModel, args...)
    return JuMP.register(mylevel_model(model), args...)
end
function JuMP.register(model::LowerModel, args...)
    return no_nlp_lower()
end
function JuMP.register(model::BilevelModel, args...)
    return no_nlp()
end

function JuMP.NLPEvaluator(
    model::UpperModel;
    _differentiation_backend::MOI.Nonlinear.AbstractAutomaticDifferentiation = MOI.Nonlinear.SparseReverseMode(),
)
    return JuMP.NLPEvaluator(mylevel_model(model), _differentiation_backend)
end
function JuMP.NLPEvaluator(
    model::LowerModel;
    _differentiation_backend::MOI.Nonlinear.AbstractAutomaticDifferentiation = MOI.Nonlinear.SparseReverseMode(),
)
    return no_nlp_lower()
end
function JuMP.NLPEvaluator(
    model::BilevelModel;
    _differentiation_backend::MOI.Nonlinear.AbstractAutomaticDifferentiation = MOI.Nonlinear.SparseReverseMode(),
)
    return no_nlp()
end
