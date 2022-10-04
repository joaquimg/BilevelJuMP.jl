#=
    NLP
    many unsafe operations here, because we are using JuMP non exposed
    methods (the ones starting with "_")
    - TODO: PIN JUMP
=#

function _has_nlp_data(model)
    return model.nlp_data !== nothing
end
function _load_nlp_data(model)
    # callen in JuMP.optimize!
    # The nlp_data is not kept in sync, so re-set it here.
    # TODO: Consider how to handle incremental solves.
    if model.nlp_data !== nothing
        MOI.set(model, MOI.NLPBlock(), JuMP._create_nlp_block_data(model))
        empty!(model.nlp_data.nlconstr_duals)
    end
end

function Base.getproperty(m::UpperModel, s::Symbol)
    if s === :nlp_data
        return mylevel_model(m).nlp_data
    else
        getfield(m, s)
    end
end
function Base.getproperty(m::LowerModel, s::Symbol)
    if s === :nlp_data
        return no_nlp_lower()
    else
        getfield(m, s)
    end
end

no_nlp() = error("Non-linear data must be passed to the Upper(.) model")
no_nlp_lower() = error("NLconstraint(s) are not allowed in the lower level")
no_nlp_lower_param() = error("NLparameter(s) are not allowed in the lower level")

JuMP._init_NLP(m::UpperModel) = JuMP._init_NLP(mylevel_model(m))
JuMP._init_NLP(m::LowerModel) = no_nlp_lower()

function JuMP._new_parameter(m::UpperModel, value::Number)
    JuMP._init_NLP(m)
    upper = m.m.upper
    nldata::JuMP._NLPData = upper.nlp_data
    push!(nldata.nlparamvalues, value)
    return JuMP.NonlinearParameter(upper, length(nldata.nlparamvalues))
end

function JuMP._new_parameter(::LowerModel, ::Number)
    no_nlp_lower_param()
end
function JuMP._new_parameter(::BilevelModel, ::Number)
    no_nlp()
end

# function JuMP.set_objective_function(m::UpperModel, func::JuMP._NonlinearExprData)
#     JuMP.set_objective_function(mylevel_model(m), func)
#     return nothing
# end
# function JuMP.set_objective_function(::LowerModel, ::JuMP._NonlinearExprData)
#     no_nlp_lower()
#     return nothing
# end
function JuMP.set_objective(
    m::UpperModel,
    sense::MOI.OptimizationSense,
    ex::JuMP._NonlinearExprData,
)
    JuMP._init_NLP(m)
    JuMP.set_objective_sense(m, sense)
    m.nlp_data.nlobj = ex
    return
end
function JuMP.set_objective(
    ::LowerModel,
    ::MOI.OptimizationSense,
    ::JuMP._NonlinearExprData,
)
    no_nlp_lower()
    return
end

function JuMP._parse_NL_expr_runtime(m::UpperModel, x, tape, parent, values)
    JuMP._parse_NL_expr_runtime(mylevel_model(m), x, tape, parent, values)
    return nothing
end

function JuMP._parse_NL_expr_runtime(
    m::UpperModel,
    x::BilevelVariableRef,
    tape,
    parent,
    values,
)
    if !in_upper(x)
        error(
            "Variable in nonlinear expression does not belong to the " *
            "upper model, it is a LowerOnly variable",
        )
    end
    JuMP._parse_NL_expr_runtime(mylevel_model(m), upper_ref(x), tape, parent, values)
    return nothing
end

# function JuMP._parse_NL_expr_runtime(
#     m::Model,
#     x::NonlinearExpression,
#     tape,
#     parent,
#     values,
# )
#     push!(tape, NodeData(SUBEXPRESSION, x.index, parent))
#     return nothing
# end

# TODO: deal with param
# function JuMP._parse_NL_expr_runtime(
#     m::JuMP.Model,
#     x::JuMP.NonlinearParameter,
#     tape,
#     parent,
#     values,
# )
#     push!(tape, JuMP.NodeData(JuMP.PARAMETER, x.index, parent))
#     return nothing
# end

# function JuMP._parse_NL_expr_runtime(
#     m::Model,
#     x::AbstractArray,
#     tape,
#     parent,
#     values,
# )
#     return error(
#         "Unexpected array $x in nonlinear expression. Nonlinear expressions may contain only scalar expressions.",
#     )
# end

# function JuMP._parse_NL_expr_runtime(
#     m::Model,
#     x::GenericQuadExpr,
#     tape,
#     parent,
#     values,
# )
#     push!(tape, NodeData(CALL, operator_to_id[:+], parent))
#     sum_parent = length(tape)
#     _parse_NL_expr_runtime(m, x.aff, tape, sum_parent, values)
#     for (xy, c) in x.terms
#         push!(tape, NodeData(CALL, operator_to_id[:*], sum_parent))
#         mult_parent = length(tape)
#         _parse_NL_expr_runtime(m, xy.a, tape, mult_parent, values)
#         _parse_NL_expr_runtime(m, xy.b, tape, mult_parent, values)
#         if !isone(c)  # Optimization: no need for * node.
#             _parse_NL_expr_runtime(m, c, tape, mult_parent, values)
#         end
#     end
#     return
# end

# function JuMP._parse_NL_expr_runtime(
#     m::Model,
#     x::GenericAffExpr,
#     tape,
#     parent,
#     values,
# )
#     push!(tape, NodeData(CALL, operator_to_id[:+], parent))
#     sum_parent = length(tape)
#     if !iszero(x.constant)
#         _parse_NL_expr_runtime(m, x.constant, tape, sum_parent, values)
#     end
#     for (v, c) in x.terms
#         if isone(c)  # Optimization: no need for * node.
#             _parse_NL_expr_runtime(m, v, tape, sum_parent, values)
#         else
#             push!(tape, NodeData(CALL, operator_to_id[:*], sum_parent))
#             mult_parent = length(tape)
#             _parse_NL_expr_runtime(m, c, tape, mult_parent, values)
#             _parse_NL_expr_runtime(m, v, tape, mult_parent, values)
#         end
#     end
#     return
# end

# function JuMP._parse_NL_expr_runtime(m::Model, x, tape, parent, values)
#     return error(
#         "Unexpected object $x (of type $(typeof(x)) in nonlinear expression.",
#     )
# end

function JuMP._parse_NL_expr_runtime(m::LowerModel, x, tape, parent, values)
    return no_nlp_lower()
end

#=

function JuMP._Derivatives.expr_to_nodedata(
    ex::VariableRef,
    nd::Vector{NodeData},
    values::Vector{Float64},
    parentid,
    r::_Derivatives.UserOperatorRegistry,
)
    push!(nd, NodeData(MOIVARIABLE, ex.index.value, parentid))
    return nothing
end

function JuMP._Derivatives.expr_to_nodedata(
    ex::NonlinearExpression,
    nd::Vector{NodeData},
    values::Vector{Float64},
    parentid,
    r::_Derivatives.UserOperatorRegistry,
)
    push!(nd, NodeData(SUBEXPRESSION, ex.index, parentid))
    return nothing
end

function JuMP._Derivatives.expr_to_nodedata(
    ex::NonlinearParameter,
    nd::Vector{NodeData},
    values::Vector{Float64},
    parentid,
    r::_Derivatives.UserOperatorRegistry,
)
    push!(nd, NodeData(PARAMETER, ex.index, parentid))
    return nothing
end

=#

#=

# Construct a _NonlinearExprData from a Julia expression.
# VariableRef objects should be spliced into the expression.
function JuMP._NonlinearExprData(m::Model, ex::Expr)
    _init_NLP(m)
    _check_expr(m, ex)
    nd, values = _Derivatives.expr_to_nodedata(ex, m.nlp_data.user_operators)
    return _NonlinearExprData(nd, values)
end
_NonlinearExprData(m::Model, ex) = _NonlinearExprData(m, :($ex + 0))

# Error if:
# 1) Unexpected expression
# 2) VariableRef doesn't match the model
function JuMP._check_expr(m::Model, ex::Expr)
    if ex.head == :ref # if we have x[1] already in there, something is wrong
        error(
            "Unrecognized expression $ex. JuMP variable objects and input coefficients should be spliced directly into expressions.",
        )
    end
    for e in ex.args
        _check_expr(m, e)
    end
    return
end
function JuMP._check_expr(m::Model, v::VariableRef)
    owner_model(v) === m || error("Variable $v does not belong to this model.")
    return
end
_check_expr(m::Model, ex) = nothing

=#

# TODO - convert into bilevel constraint ref
# ConstraintRef(
#     $esc_m,
#     NonlinearConstraintIndex(length($esc_m.nlp_data.nlconstr)),
#     ScalarShape(),
# )
