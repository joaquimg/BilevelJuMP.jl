#=
    Helpers
=#

const VI = MOI.VariableIndex
const CI = MOI.ConstraintIndex

const GT = MOI.GreaterThan
const LT = MOI.LessThan
const ET = MOI.EqualTo

const SCALAR_SETS =
    Union{MOI.GreaterThan{Float64},MOI.LessThan{Float64},MOI.EqualTo{Float64}}

const VECTOR_SETS = Union{MOI.SecondOrderCone,MOI.RotatedSecondOrderCone}

function _appush!(col, element::AbstractVector)
    append!(col, element)
    return nothing
end

function _appush!(col, element)
    push!(col, element)
    return nothing
end

nothing_to_nan(val) = ifelse(val === nothing, NaN, val)

#=
    Information caches
=#

# dual variable info
mutable struct BilevelConstraintInfo{T<:Union{Float64,Vector{Float64}}}
    level::Level
    start::T
    upper::T
    lower::T
    function BilevelConstraintInfo{Float64}(level)
        return new(level, NaN, NaN, NaN)
    end
    function BilevelConstraintInfo{Vector{Float64}}(level, N::Integer)
        return new(level, fill(NaN, N), fill(NaN, N), fill(NaN, N))
    end
end

mutable struct BilevelVariableInfo{T<:Union{Float64,Vector{Float64}}}
    level::Level
    upper::T
    lower::T
    function BilevelVariableInfo(level)
        return new{Float64}(level, NaN, NaN)
    end
    function BilevelVariableInfo(level, N::Integer)
        return new{Vector{Float64}}(level, fill(NaN, N), fill(NaN, N))
    end
end

function BilevelVariableInfo(_info::BilevelConstraintInfo{T}) where {T}
    if isa(_info.upper, Number)
        info = BilevelVariableInfo(DUAL_OF_LOWER)
        info.lower = _info.lower
        info.upper = _info.upper
    else
        info = BilevelVariableInfo(DUAL_OF_LOWER, length(_info.upper))
        info.lower .= _info.lower
        info.upper .= _info.upper
    end
    return info
end

#=
    Complement
=#

struct Complement#{M1 <: MOI.ModelLike, M2 <: MOI.ModelLike, F, S}
    is_vec::Any
    # primal::M1
    constraint::Any
    func_w_cte::Any#::F
    set_w_zero::Any#::S
    # dual::M2
    variable::Any#::VI
    # var_set#::S2
end

function get_canonical_complements(primal_model, primal_dual_map)
    map = primal_dual_map.primal_con_dual_var
    out = Complement[]
    for ci in keys(map)
        con = get_canonical_complement(primal_model, map, ci)
        push!(out, con)
    end
    return out
end

function get_canonical_complement(
    primal_model,
    map,
    ci::CI{F,S},
) where {F,S<:VECTOR_SETS}
    T = Float64
    func = MOI.copy(MOI.get(primal_model, MOI.ConstraintFunction(), ci))::F
    set = MOI.copy(MOI.get(primal_model, MOI.ConstraintSet(), ci))::S
    dim = MOI.dimension(set)
    # vector sets have no constant
    # for i in 1:dim
    #     func.constant[i] = Dualization.set_dot(i, set, T) *
    #         Dualization.get_scalar_term(primal_model, ci, i)
    # end
    # todo - set dot on function
    con = Complement(true, ci, func, _set_with_zero(set), map[ci])
    return con
end

function get_canonical_complement(
    primal_model,
    map,
    ci::CI{F,S},
) where {F,S<:SCALAR_SETS}
    T = Float64
    func = MOI.copy(MOI.get(primal_model, MOI.ConstraintFunction(), ci))::F
    set = MOI.copy(MOI.get(primal_model, MOI.ConstraintSet(), ci))::S
    constant =
        Dualization.set_dot(1, set, T) *
        Dualization.get_scalar_term(primal_model, ci)[]
    if F == MOI.VariableIndex
        func = MOIU.operate(+, T, func, constant)
    else
        func.constant = constant
    end
    # todo - set dot on function
    con = Complement(false, ci, func, _set_with_zero(set), map[ci][1])
    return con
end

function _set_with_zero(set::S) where {S<:SCALAR_SETS}
    return S(0.0)
end

function _set_with_zero(set)
    return MOI.copy(set)
end

#=
    BilevelSolverMode
=#

abstract type AbstractBilevelSolverMode{T} end

mutable struct NoMode{T} <: AbstractBilevelSolverMode{T} end

function reset!(::AbstractBilevelSolverMode)
    return nothing
end

ignore_dual_objective(::AbstractBilevelSolverMode{T}) where {T} = true

function accept_vector_set(
    mode::AbstractBilevelSolverMode{T},
    con::Complement,
) where {T}
    if con.is_vec
        error(
            "Set $(typeof(con.set_w_zero)) is not accepted when solution method is $(typeof(mode))",
        )
    end
    return nothing
end

#=
    ComplementBoundCache used in Big-M
=#

struct ComplementBoundCache
    # internal usage
    upper::Dict{VI,BilevelVariableInfo}
    lower::Dict{VI,BilevelVariableInfo}
    ldual::Dict{CI,BilevelConstraintInfo}
    # full map
    map::Dict{VI,BilevelVariableInfo}
    function ComplementBoundCache()
        return new(
            Dict{VI,BilevelVariableInfo}(),
            Dict{VI,BilevelVariableInfo}(),
            Dict{CI,BilevelConstraintInfo}(),
            Dict{VI,BilevelVariableInfo}(),
        )
    end
end

function build_full_map!(
    mode,
    upper_idxmap,
    lower_idxmap,
    lower_dual_idxmap,
    lower_primal_dual_map,
)
    return nothing
end

function _build_bound_map!(
    mode::ComplementBoundCache,
    upper_idxmap,
    lower_idxmap,
    lower_dual_idxmap,
    lower_primal_dual_map,
)
    empty!(mode.map)
    for (k, v) in mode.upper
        mode.map[upper_idxmap[k]] = v
    end
    for (k, v) in mode.lower
        mode.map[lower_idxmap[k]] = v
    end
    for (k, v) in mode.ldual
        vec = lower_primal_dual_map.primal_con_dual_var[k]#[1] # TODO check this scalar
        for var in vec
            mode.map[lower_dual_idxmap[var]] = BilevelVariableInfo(v)
        end
    end
    return nothing
end

#=
    Main bilevel builder
=#

function build_bilevel(
    upper::MOI.ModelLike,
    lower::MOI.ModelLike,
    upper_to_lower_link::Dict{VI,VI},
    upper_variables::Vector{VI},
    mode,
    upper_var_to_lower_ctr::Dict{VI,CI} = Dict{VI,CI}();
    copy_names::Bool = false,
    pass_start::Bool = false,
)

    # Start with an empty problem
    moi_mode = MOIU.AUTOMATIC
    m = MOIU.CachingOptimizer(
        MOIU.UniversalFallback(MOIU.Model{Float64}()),
        moi_mode,
    )

    #=
        Create Lower DUAL level model
    =#
    # dualize the second level
    dual_problem = Dualization.dualize(
        lower;
        dual_names = Dualization.DualNames("dual_", "dual_"),
        variable_parameters = upper_variables,
        ignore_objective = ignore_dual_objective(mode),
        consider_constrained_variables = false,
    )
    # the model
    lower_dual = dual_problem.dual_model
    # the mapping from primal to dual references
    lower_primal_dual_map = dual_problem.primal_dual_map

    #=
        Pass Upper level model
    =#

    # keys are from src (upper), values are from dest (m)
    upper_idxmap = MOIU.default_copy_to(m, upper)
    if copy_names
        pass_names(m, upper, upper_idxmap)
    end

    #=
        Pass Lower level model
    =#

    handle_lower_objective_sense(lower)

    # cache and delete lower objective
    if !ignore_dual_objective(mode)
        # get primal obj
        type_primal_obj = MOI.get(lower, MOI.ObjectiveFunctionType())
        @assert type_primal_obj !== nothing
        lower_primal_obj =
            MOI.get(lower, MOI.ObjectiveFunction{type_primal_obj}())
        # deepcopy and delete dual obj
        # MOI.set(lower, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(), MOI.ScalarAffineFunction(MOI.ScalarAffineTerm{Float64}[], 0.0))
    end

    # initialize map from (original) lower model to single model (m)
    lower_idxmap = MOIU.IndexMap()
    for (upper_key, lower_val) in upper_to_lower_link
        lower_idxmap[lower_val] = upper_idxmap[upper_key]
    end

    # append the second level primal
    append_to(m, lower, lower_idxmap; allow_single_bounds = true)
    if copy_names
        pass_names(m, lower, lower_idxmap)
    end

    #=
        Pass Dual of Lower level model
    =#

    # initialize map to lower level model
    if !ignore_dual_objective(mode)
        # get dual obj
        tp_dual_obj = MOI.get(lower_dual, MOI.ObjectiveFunctionType())
        @assert tp_dual_obj !== nothing
        lower_dual_obj =
            MOI.get(lower_dual, MOI.ObjectiveFunction{tp_dual_obj}())
        # delete dual obj
        # MOI.set(lower_dual, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(), MOI.ScalarAffineFunction(MOI.ScalarAffineTerm{Float64}[], 0.0))
    end

    # initialize map from *dual lower* model to single model (m)
    lower_dual_idxmap = MOIU.IndexMap()
    # 1) for QP's there are dual variable that are linked to:
    # 1.1) primal variables
    for (lower_primal_var_key, lower_dual_quad_slack_val) in
        lower_primal_dual_map.primal_var_dual_quad_slack
        lower_dual_idxmap[lower_dual_quad_slack_val] =
            lower_idxmap[lower_primal_var_key]
    end
    # 1.2) and to upper level variable which are lower level parameters
    for (lower_primal_param_key, lower_dual_param_val) in
        lower_primal_dual_map.primal_parameter
        lower_dual_idxmap[lower_dual_param_val] =
            lower_idxmap[lower_primal_param_key]
    end
    # 2) Dual variables might appear in the upper level
    for (upper_var, lower_con) in upper_var_to_lower_ctr
        var = lower_primal_dual_map.primal_con_dual_var[lower_con][1] # TODO check this scalar
        lower_dual_idxmap[var] = upper_idxmap[upper_var]
    end

    # append the second level dual
    append_to(m, lower_dual, lower_dual_idxmap)
    if copy_names
        pass_names(m, lower_dual, lower_dual_idxmap)
    end

    #=
        Additional Optimiality conditions (to complete the KKT)
    =#

    # build map bound map for FortunyAmatMcCarlMode
    build_full_map!(
        mode,
        upper_idxmap,
        lower_idxmap,
        lower_dual_idxmap,
        lower_primal_dual_map,
    )

    if ignore_dual_objective(mode)
        # complementary slackness
        comps = get_canonical_complements(lower, lower_primal_dual_map)
        for comp in comps
            if !is_equality(comp.set_w_zero)
                accept_vector_set(mode, comp)
                add_complement(
                    mode,
                    m,
                    comp,
                    lower_idxmap,
                    lower_dual_idxmap,
                    copy_names,
                    pass_start,
                )
            else
                # feasible equality constraints always satisfy complementarity
            end
        end
    else # strong duality
        add_strong_duality(
            mode,
            m,
            lower_primal_obj,
            lower_dual_obj,
            lower_idxmap,
            lower_dual_idxmap,
        )
    end
    add_aggregate_constraints(m, mode, copy_names)

    return m,
    upper_idxmap,
    lower_idxmap,
    lower_primal_dual_map,
    lower_dual_idxmap
end

add_aggregate_constraints(m, mode, copy_names) = nothing

function _add_aggregate_constraints(
    m,
    func::F,
    eps,
    idx,
    copy_names,
) where {F<:MOI.ScalarQuadraticFunction{T}} where {T}
    prod_f1 = MOIU.operate(-, T, func, eps)
    c1 = MOIU.normalize_and_add_constraint(m, prod_f1, MOI.LessThan{T}(0.0))
    # _appush!(out_ctr, c1)
    if true # comp.is_vec
        prod_f2 = MOIU.operate(+, T, func, eps)
        c2 = MOIU.normalize_and_add_constraint(
            m,
            prod_f2,
            MOI.GreaterThan{T}(0.0),
        )
        # _appush!(out_ctr, c2)
    end
    if copy_names
        MOI.set(m, MOI.ConstraintName(), c1, "compl_prod_agg1_$idx")
        if true # comp.is_vec
            MOI.set(m, MOI.ConstraintName(), c2, "compl_prod_agg2_$idx")
        end
    end
    return nothing
end

is_equality(set::S) where {S<:MOI.AbstractSet} = false

is_equality(set::MOI.EqualTo{T}) where {T} = true

is_equality(set::MOI.Zeros) = true

function handle_lower_objective_sense(lower::MOI.ModelLike)
    lower_objective_sense = MOI.get(lower, MOI.ObjectiveSense())
    if lower_objective_sense == MOI.FEASIBILITY_SENSE
        throw(
            ErrorException(
                "Lower level models with objective_sense: " *
                lower_objective_sense *
                " are not supported.",
            ),
        )
    end
    return
end
