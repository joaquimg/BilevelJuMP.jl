const SVF = MOI.SingleVariable
const VVF = MOI.VectorOfVariables
const SAF{T} = MOI.ScalarAffineFunction{T}
const VAF{T} = MOI.VectorAffineFunction{T}

const VI = MOI.VariableIndex
const CI = MOI.ConstraintIndex

const GT = MOI.GreaterThan
const LT = MOI.LessThan
const ET = MOI.EqualTo

const SCALAR_SETS = Union{
    MOI.GreaterThan{Float64},
    MOI.LessThan{Float64},
    MOI.EqualTo{Float64},
}

const VECTOR_SETS = Union{
    MOI.SecondOrderCone,
    MOI.RotatedSecondOrderCone,
}

# dual variable info
mutable struct BilevelConstraintInfo{T<:Union{Float64, Vector{Float64}}}
    level::Level
    start::T
    upper::T
    lower::T
    function BilevelConstraintInfo{Float64}(level)
        new(level, NaN, NaN, NaN)
    end
    function BilevelConstraintInfo{Vector{Float64}}(level, N::Integer)
        new(level, fill(NaN, N), fill(NaN, N), fill(NaN, N))
    end
end

mutable struct BilevelVariableInfo{T<:Union{Float64, Vector{Float64}}}
    level::Level
    upper::T
    lower::T
    function BilevelVariableInfo(level)
        new{Float64}(level, NaN, NaN)
    end
    function BilevelVariableInfo(level, N::Integer)
        new{Vector{Float64}}(level, fill(NaN, N), fill(NaN, N))
    end
end
function BilevelVariableInfo(_info::BilevelConstraintInfo{T}) where T
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

struct Complement#{M1 <: MOI.ModelLike, M2 <: MOI.ModelLike, F, S}
    is_vec
    # primal::M1
    constraint
    func_w_cte#::F
    set_w_zero#::S
    # dual::M2
    variable#::VI
    # var_set#::S2
end

abstract type AbstractBilevelSolverMode{T} end

mutable struct NoMode{T} <: AbstractBilevelSolverMode{T}
end

mutable struct SOS1Mode{T} <: AbstractBilevelSolverMode{T}
    function SOS1Mode()
        return new{Float64}()
    end
end

mutable struct ComplementMode{T} <: AbstractBilevelSolverMode{T}
    with_slack::Bool
    function ComplementMode(;with_slack = false)
        return new{Float64}(with_slack)
    end
end

mutable struct ProductMode{T} <: AbstractBilevelSolverMode{T}
    epsilon::T
    with_slack::Bool
    aggregation_group::Int # only useful in mixed mode
    function_cache::Union{Nothing, MOI.AbstractScalarFunction}
    function ProductMode(
        eps::T=zero(Float64);
        with_slack::Bool = false,
        aggregation_group = nothing
    ) where T<:Float64 # Real
        @assert aggregation_group === nothing || aggregation_group >= 1
        # nothing means individualized
        # positive integers point to their numbers
        return new{Float64}(
            eps,
            with_slack,
            aggregation_group === nothing ? 0 : aggregation_group,
            nothing,
            )
    end
end

@enum IndicatorSetting ZERO_ONE ZERO_ZERO ONE_ONE

mutable struct IndicatorMode{T} <: AbstractBilevelSolverMode{T}
    mode::IndicatorSetting
    function IndicatorMode()
        return new{Float64}(ONE_ONE)
    end
end

struct ComplementBoundCache
    # internal usage
    upper::Dict{VI, BilevelVariableInfo}
    lower::Dict{VI, BilevelVariableInfo}
    ldual::Dict{CI, BilevelConstraintInfo}
    # full map
    map::Dict{VI, BilevelVariableInfo}
    function ComplementBoundCache()
        return new(
            Dict{VI, BilevelVariableInfo}(),
            Dict{VI, BilevelVariableInfo}(),
            Dict{CI, BilevelConstraintInfo}(),
            Dict{VI, BilevelVariableInfo}(),
        )
    end
end

mutable struct FortunyAmatMcCarlMode{T} <: AbstractBilevelSolverMode{T}
    with_slack::Bool
    primal_big_M::Float64
    dual_big_M::Float64
    cache::ComplementBoundCache
    function FortunyAmatMcCarlMode(;with_slack = false,
        primal_big_M = Inf, dual_big_M = Inf)
        return new{Float64}(
            with_slack,
            primal_big_M,
            dual_big_M,
            ComplementBoundCache()
        )
    end
end

mutable struct MixedMode{T} <: AbstractBilevelSolverMode{T}
    default::AbstractBilevelSolverMode{T}
    constraint_mode_map_c::Dict{CI, AbstractBilevelSolverMode{T}}
    constraint_mode_map_v::Dict{VI, AbstractBilevelSolverMode{T}}
    cache::ComplementBoundCache
    function_cache::Dict{Int,MOI.AbstractScalarFunction}
    function MixedMode(;default = SOS1Mode())
        return new{Float64}(
            default,
            Dict{CI, AbstractBilevelSolverMode{Float64}}(),
            Dict{VI, AbstractBilevelSolverMode{Float64}}(),
            ComplementBoundCache(),
            Dict{Int,MOI.AbstractScalarFunction}()
        )
    end
end

function reset!(::AbstractBilevelSolverMode)
    return nothing
end
function reset!(mode::FortunyAmatMcCarlMode)
    mode.cache = ComplementBoundCache()
    return nothing
end
function reset!(mode::ProductMode)
    mode.function_cache = nothing
    return nothing
end
function reset!(mode::MixedMode)
    mode.cache = ComplementBoundCache()
    mode.function_cache = Dict{Int,MOI.AbstractScalarFunction}()
    for v in values(mode.constraint_mode_map_c)
        reset!(v)
    end
    for v in values(mode.constraint_mode_map_v)
        reset!(v)
    end
    return nothing
end

function appush!(col, element::AbstractVector)
    append!(col, element)
    return nothing
end
function appush!(col, element)
    push!(col, element)
    return nothing
end

function build_full_map!(mode,
    upper_to_m_idxmap, lower_to_m_idxmap, lower_dual_idxmap, lower_primal_dual_map)
    return nothing
end
function build_full_map!(mode::FortunyAmatMcCarlMode,
        upper_to_m_idxmap, lower_to_m_idxmap, lower_dual_idxmap, lower_primal_dual_map)
    _build_bound_map!(mode.cache,
        upper_to_m_idxmap, lower_to_m_idxmap, lower_dual_idxmap, lower_primal_dual_map)
    return nothing
end
function build_full_map!(mode::MixedMode,
    upper_idxmap, lower_idxmap, lower_dual_idxmap, lower_primal_dual_map)
_build_bound_map!(mode.cache,
    upper_idxmap, lower_idxmap, lower_dual_idxmap, lower_primal_dual_map)
return nothing
end
function _build_bound_map!(mode::ComplementBoundCache,
    upper_to_m_idxmap, lower_to_m_idxmap, lower_dual_idxmap, lower_primal_dual_map)
    empty!(mode.map)
    for (k,v) in mode.upper
        mode.map[upper_to_m_idxmap[k]] = v
    end
    for (k,v) in mode.lower
        mode.map[lower_to_m_idxmap[k]] = v
    end
    for (k,v) in mode.ldual
        vec = lower_primal_dual_map.primal_con_dual_var[k]#[1] # TODO check this scalar
        for var in vec
            mode.map[lower_dual_idxmap[var]] = BilevelVariableInfo(v)
        end
    end
    return nothing
end

mutable struct StrongDualityMode{T} <: AbstractBilevelSolverMode{T}
    inequality::Bool
    epsilon::T
    function StrongDualityMode(eps::T=zero(Float64); inequality = true) where T
        return new{T}(inequality, eps)
    end
end

ignore_dual_objective(::AbstractBilevelSolverMode{T}) where T = true
ignore_dual_objective(::StrongDualityMode{T}) where T = false

function accept_vector_set(mode::AbstractBilevelSolverMode{T}, con::Complement) where T
    if con.is_vec
        error("Set $(typeof(con.set_w_zero)) is not accepted when solution method is $(typeof(mode))")
    end
    return nothing
end
accept_vector_set(::ProductMode{T}, ::Complement) where T = nothing
accept_vector_set(::MixedMode{T}, ::Complement) where T = nothing

function get_canonical_complements(primal_model, primal_dual_map)
    map = primal_dual_map.primal_con_dual_var
    out = Complement[]
    for ci in keys(map)
        con = get_canonical_complement(primal_model, map, ci)
        push!(out, con)
    end
    return out
end
function get_canonical_complement(primal_model, map,
    ci::CI{F,S}) where {F, S<:VECTOR_SETS}
    T = Float64
    func = MOI.copy(MOI.get(primal_model, MOI.ConstraintFunction(), ci))::F
    set = MOI.copy(MOI.get(primal_model, MOI.ConstraintSet(), ci))::S
    dim = MOI.dimension(set)
    # vector sets have no constant
    # for i in 1:dim
    #     func.constant[i] = Dualization.set_dot(i, set, T) *
    #         Dualization.get_scalar_term(primal_model, i, ci)
    # end
    # todo - set dot on function
    con = Complement(true, ci, func, set_with_zero(set), map[ci])
    return con
end
function get_canonical_complement(primal_model, map,
    ci::CI{F,S}) where {F, S<:SCALAR_SETS}
    T = Float64
    func = MOI.copy(MOI.get(primal_model, MOI.ConstraintFunction(), ci))::F
    set = MOI.copy(MOI.get(primal_model, MOI.ConstraintSet(), ci))::S
    constant = Dualization.set_dot(1, set, T) *
        Dualization.get_scalar_term(primal_model, 1, ci)
    if F == MOI.SingleVariable
        func = MOIU.operate(+, T, func, constant)
    else
        func.constant = constant
    end
    # todo - set dot on function
    con = Complement(false, ci, func, set_with_zero(set), map[ci][1])
    return con
end

function set_with_zero(set::S) where {S<:SCALAR_SETS}
    return S(0.0)
end
function set_with_zero(set)
    return MOI.copy(set)
end

function build_bilevel(
    upper::MOI.ModelLike, 
    lower::MOI.ModelLike,
    upper_to_lower_var_indices::Dict{VI,VI}, 
    lower_var_indices_of_upper_vars::Vector{VI},
    mode,
    upper_var_to_lower_ctr::Dict{VI,CI} = Dict{VI,CI}();
    copy_names::Bool = false,
    pass_start::Bool = false,
    linearize_bilinear_upper_terms::Bool = false
    )

    # Start with an empty problem
    m = MOIU.CachingOptimizer(MOIU.UniversalFallback(MOIU.Model{Float64}()), MOIU.AUTOMATIC)

    #=
        Create Lower DUAL level model
    =#
    # TODO if linearizing bilinear terms the lower dual model must be built using standard form!
    #   (for now requiring that lower model is in standard form)
    # dualize the second level
    dual_problem = Dualization.dualize(lower,
        dual_names = Dualization.DualNames("dual_","dual_"),
        variable_parameters = lower_var_indices_of_upper_vars,
        ignore_objective = ignore_dual_objective(mode))
    # the model
    lower_dual = dual_problem.dual_model
    # the mapping from primal to dual references
    lower_primal_dual_map = dual_problem.primal_dual_map

    #=
        Pass Upper level model
    =#

    # key are from src, value are from dest
    upper_to_m_idxmap = MOIU.default_copy_to(m, upper, copy_names)
    if copy_names
        pass_names(m, upper, upper_to_m_idxmap)
    end

    #=
        Pass Lower level model
    =#

    handle_lower_objective_sense(lower)

    # cache and delete lower objective
    if !BilevelJuMP.ignore_dual_objective(mode)
        # get primal obj
        type_primal_obj = MOI.get(lower, MOI.ObjectiveFunctionType())
        @assert type_primal_obj !== nothing
        lower_primal_obj = MOI.get(lower, MOI.ObjectiveFunction{type_primal_obj}())
        # deepcopy and delete dual obj
        # MOI.set(lower, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(), MOI.ScalarAffineFunction(MOI.ScalarAffineTerm{Float64}[], 0.0))
    end

    # initialize map to lower level model
    lower_to_m_idxmap = MOIU.IndexMap()
    lower_to_upper_var_indices = Dict{MOI.VariableIndex, MOI.VariableIndex}()
    for (upper_key, lower_val) in upper_to_lower_var_indices
        lower_to_m_idxmap[lower_val] = upper_to_m_idxmap[upper_key]
        lower_to_upper_var_indices[lower_val] = upper_key
    end

    # append the second level primal
    append_to(m, lower, lower_to_m_idxmap, copy_names, allow_single_bounds = true)
    if copy_names
        pass_names(m, lower, lower_to_m_idxmap)
    end

    #=
        Pass Dual of Lower level model
    =#

    # initialize map to lower level model
    if !ignore_dual_objective(mode)
        # get dual obj
        tp_dual_obj = MOI.get(lower_dual, MOI.ObjectiveFunctionType())
        @assert tp_dual_obj !== nothing
        lower_dual_obj = MOI.get(lower_dual, MOI.ObjectiveFunction{tp_dual_obj}())
        # delete dual obj
        # MOI.set(lower_dual, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(), MOI.ScalarAffineFunction(MOI.ScalarAffineTerm{Float64}[], 0.0))
    end

    # initialize map from lower level dual variables to m indices
    lower_dual_idxmap = MOIU.IndexMap()
    # for lower level QP's there are lower dual variables that are tied to:
    # lower primal variables
    for (lower_primal_var_key, lower_dual_quad_slack_val) in lower_primal_dual_map.primal_var_dual_quad_slack
        lower_dual_idxmap[lower_dual_quad_slack_val] = lower_to_m_idxmap[lower_primal_var_key]
    end
    # and to upper level variables (which are lower level parameters)
    for (lower_primal_param_key, lower_dual_param_val) in lower_primal_dual_map.primal_parameter
        lower_dual_idxmap[lower_dual_param_val] = lower_to_m_idxmap[lower_primal_param_key]
    end
    # lower level dual variable -> upper level variable
    # and the reverse map
    bilinear_upper_primal_lower_dual = MOIU.IndexMap()
    for (upper_var, lower_con) in upper_var_to_lower_ctr
        var = lower_primal_dual_map.primal_con_dual_var[lower_con][1] # TODO check this scalar
        lower_dual_idxmap[var] = upper_to_m_idxmap[upper_var]
        bilinear_upper_primal_lower_dual[upper_to_m_idxmap[upper_var]] = var
    end

    # append the second level dual
    append_to(m, lower_dual, lower_dual_idxmap, copy_names)
    if copy_names
        pass_names(m, lower_dual, lower_dual_idxmap)
    end

    #=
        Check if Upper bilinear terms of the form Upper primal * Lower dual exist, and can be linearized
    =#
    # TODO implement for bilinear products in constraints (only checking upper objective so far)

    if linearize_bilinear_upper_terms 
        if MOI.get(upper, MOI.ObjectiveFunctionType()) <: MOI.ScalarQuadraticFunction &&
            !isempty(upper_var_to_lower_ctr)
            
            linearize = true
            # check lower constraint types and if not just equality and singlevariable bounds then linearize = false and @warn
            lower_con_types = Set(MOI.get(lower, MOI.ListOfConstraints()))
            standard_form_con_types = Set([
                (MOI.ScalarAffineFunction{Float64}, MOI.EqualTo{Float64}),
                (MOI.SingleVariable, MOI.GreaterThan{Float64}),
                (MOI.SingleVariable, MOI.LessThan{Float64})
            ])

            if !(issubset(lower_con_types, standard_form_con_types))
                linearize = false
                @warn("The lower model must be in standard form to linearize bilinear terms, i.e. the constraint types must be a subset of $(standard_form_con_types). Skipping linearization process.")
            else
                A_N, bilinear_upper_dual_to_quad_term, bilinear_upper_dual_to_lower_primal, lower_primal_var_to_lower_con = 
                    check_upper_objective_for_bilinear_linearization(upper, upper_to_lower_var_indices, upper_var_to_lower_ctr)

                if isempty(A_N)
                    @warn("No bilinear products of lower level dual and primal variables found in upper level objective. Skipping linearization process.")
                    linearize = false
                else
                    AB_N, B, lower_obj_terms, lower_obj_type_handled = 
                        get_lower_obj_coefs_of_upper_times_lower_primals(lower, lower_var_indices_of_upper_vars, A_N)
                    if !lower_obj_type_handled
                        @warn("Linearizing bilinear terms does not handle lower level objective type $(MOI.get(lower, MOI.ObjectiveFunctionType())). Skipping linearization process.")
                        linearize = false
                    end
                end
            end
        else
            @warn("Upper objective must be quadratic and there must lower dual variables in the upper model to linearize bilinear terms. Skipping linearization process.")
            linearize = false
        end

        if linearize
            U, V, w = standard_form(lower, upper_var_indices=lower_var_indices_of_upper_vars)
            upper_obj_func_quad_terms = MOI.get(upper, MOI.ObjectiveFunction{MOI.get(upper, MOI.ObjectiveFunctionType())}()).quadratic_terms
            linearizations = nothing
            m_objective = MOI.get(m, MOI.ObjectiveFunction{MOI.get(m, MOI.ObjectiveFunctionType())}())
            bilinear_upper_quad_term_to_m_quad_term = Dict{MOI.ScalarQuadraticTerm, MOI.ScalarQuadraticTerm}()

            for term in m_objective.quadratic_terms
                mset = Set([term.variable_index_1, term.variable_index_2])
                for upper_term in upper_obj_func_quad_terms
                    uset = Set([
                        upper_to_m_idxmap[upper_term.variable_index_1],
                        upper_to_m_idxmap[upper_term.variable_index_2]
                    ])
                    if uset == mset
                        bilinear_upper_quad_term_to_m_quad_term[upper_term] = term
                    end
                end
            end

            # TODO check for integer x * continuous y, for now assuming continuous x conditions

            J_U = Int[]
            N_U = Int[]
            for (upper_var, lower_con) in upper_var_to_lower_ctr  # equivalent to set A with pairs (j,n) : A_jn ≠ 0
                j = lower_con.value
                n = bilinear_upper_dual_to_lower_primal[upper_var].value
                if !(n in keys(bilinear_upper_dual_to_lower_primal)) continue end  # user defined DualOf but did not use it in UL objective
                rows, cols = find_connected_rows_cols(V, j, n, skip_1st_col_check=!(isempty(AB_N)))
                push!(J_U, rows...)
                push!(N_U, cols...)
            end
            #= 
                Case without x_m * y_n in LL objective for all y_n in A_N (set of bilinear UL objective terms of form λ_j * y_n)
            =#
            if isempty(AB_N)
                @debug("set AB_N is empty")

                conditions_passed = check_empty_AB_N_conditions(J_U, U, N_U, B)

                if conditions_passed
                    linearizations = linear_terms_for_empty_AB(
                        lower,
                        upper_var_to_lower_ctr,
                        bilinear_upper_dual_to_lower_primal,
                        V,
                        w,
                        bilinear_upper_dual_to_quad_term,
                        lower_obj_terms,
                        lower_to_m_idxmap,
                        lower_primal_dual_map,
                        lower_dual_idxmap
                    )
                    if isnothing(linearizations)
                        @warn("Unable to linearize bilinear terms due to underdertermined system of equations. Skipping linearization process.")
                    end
                else
                    @warn("Required conditions for linearization not met. Skipping linearization process.")
                end
            else  # AB_N is not empty
                @debug("set AB_N is NOT empty")
                
                # TODO input flag for checking for blocks? (to save time)
                num_blocks, rows, cols = find_blocks(V, U)

                conditions_passed = Bool[]
                nrows, ncols = size(V)
                for n in 1:num_blocks
                    # TODO can skip blocks that are not linked to bilinear terms?
                    Vblock = spzeros(nrows, ncols)
                    Vblock[rows[n],:] = V[rows[n],:]
                    check = check_non_empty_AB_N_conditions(
                        intersect(J_U, rows[n]), U, intersect(N_U, cols[n]), intersect(A_N, cols[n]), B, Vblock, 
                        lower_primal_var_to_lower_con, upper_var_to_lower_ctr,
                        bilinear_upper_dual_to_quad_term, 
                        bilinear_upper_dual_to_lower_primal)
                    push!(conditions_passed, check)
                end
                # recover memory
                rows, cols = nothing, nothing
                @debug("Done looping over V blocks")
                
                if all(conditions_passed)
                    linearizations = linear_terms_for_non_empty_AB(
                        lower,
                        upper_var_to_lower_ctr,
                        bilinear_upper_dual_to_lower_primal,
                        V,
                        w,
                        A_N,
                        bilinear_upper_dual_to_quad_term,
                        lower_obj_terms,
                        lower_to_m_idxmap,
                        lower_primal_dual_map,
                        lower_dual_idxmap
                    )
                    if isnothing(linearizations)
                        @warn("Unable to linearize bilinear terms due to underdertermined system of equations. Skipping linearization process.")
                    end
                else
                    @warn("Required conditions for linearization not met. Skipping linearization process.")
                end

            end

            # LINEARIZATION
            # TODO switch signs with MAX sense?

            if !(isnothing(linearizations))
                # set m's objective by replacing quadratic terms with linearizations
                mobj = deepcopy(m_objective)
                quadratic_terms = mobj.quadratic_terms  # TODO keep quadratic terms that have not been linearized
                c = mobj.constant
                affine_terms = mobj.affine_terms
                cvb = collect(values(bilinear_upper_quad_term_to_m_quad_term))
                new_objective = deepcopy(m_objective)
                if Set(cvb) == Set(quadratic_terms)
                    @info("Replacing bilinear lower dual * lower primal terms in upper objective with linear terms.")
                    new_objective = MOI.ScalarAffineFunction{Float64}(
                        append!(affine_terms, linearizations),
                        c
                    )
                    MOI.set(m, MOI.ObjectiveFunction{MOI.ScalarAffineFunction}(), new_objective)
                else
                    @warn("Unable to linearize bilinear terms due to mis-matched quadratic terms in the upper level and single level models. Skipping linearization process.")
                end
            end

        end # if linearize
    end  # if linearize_bilinear_upper_terms

    #=
        Additional Optimiality conditions (to complete the KKT)
    =#

    # build map bound map for FortunyAmatMcCarlMode
    build_full_map!(mode,
    upper_to_m_idxmap, lower_to_m_idxmap, lower_dual_idxmap, lower_primal_dual_map)

    if ignore_dual_objective(mode)
        # complementary slackness
        comps = get_canonical_complements(lower, lower_primal_dual_map)
        for comp in comps
            if !is_equality(comp.set_w_zero)
                accept_vector_set(mode, comp)
                add_complement(mode, m, comp,
                    lower_to_m_idxmap, lower_dual_idxmap, copy_names, pass_start)
            else
                # feasible equality constraints always satisfy complementarity
            end
        end
    else # strong duality
        add_strong_duality(mode, m, lower_primal_obj, lower_dual_obj, lower_to_m_idxmap, lower_dual_idxmap)
    end
    add_aggregate_constraints(m, mode, copy_names)

    return m, upper_to_m_idxmap, lower_to_m_idxmap, lower_primal_dual_map, lower_dual_idxmap
end

function add_strong_duality(mode::StrongDualityMode{T}, m, primal_obj, dual_obj,
    idxmap_primal, idxmap_dual) where T

    primal = MOIU.map_indices.(Ref(idxmap_primal), primal_obj)
    dual   = MOIU.map_indices.(Ref(idxmap_dual), dual_obj)

    func = MOIU.operate(-, T, primal, dual)

    if !mode.inequality
        c = MOIU.normalize_and_add_constraint(m, func, MOI.EqualTo(zero(T)))
        MOI.set(m, MOI.ConstraintName(), c, "lower_strong_duality")
        return CI[c]
    else
        func_up = MOIU.operate(-, T, func, mode.epsilon)
        c_up = MOIU.normalize_and_add_constraint(m, func_up, MOI.LessThan(zero(T)))
        MOI.set(m, MOI.ConstraintName(), c_up, "lower_strong_duality_up")

        func_lo = MOIU.operate(+, T, func, mode.epsilon)
        c_lo = MOIU.normalize_and_add_constraint(m, func_lo, MOI.GreaterThan(zero(T)))
        MOI.set(m, MOI.ConstraintName(), c_lo, "lower_strong_duality_lo")

        return CI[c_up, c_lo]
    end
end

add_aggregate_constraints(m, mode, copy_names) = nothing

function _add_aggregate_constraints(
    m, func::F, eps, idx, copy_names
) where F<:MOI.ScalarQuadraticFunction{T} where T
    prod_f1 = MOIU.operate(-, T, func, eps)
    c1 = MOIU.normalize_and_add_constraint(m,
        prod_f1,
        MOI.LessThan{T}(0.0))
    # appush!(out_ctr, c1)
    if true # comp.is_vec
        prod_f2 = MOIU.operate(+, T, func, eps)
        c2 = MOIU.normalize_and_add_constraint(m,
            prod_f2,
            MOI.GreaterThan{T}(0.0))
        # appush!(out_ctr, c2)
    end
    if copy_names
        MOI.set(m, MOI.ConstraintName(), c1, "compl_prod_agg1_$idx")
        if true # comp.is_vec
            MOI.set(m, MOI.ConstraintName(), c2, "compl_prod_agg2_$idx")
        end
    end
    return nothing
end
function add_aggregate_constraints(m, mode::ProductMode, copy_names)
    if mode.function_cache === nothing
        return nothing
    end
    _add_aggregate_constraints(
        m, mode.function_cache, mode.epsilon, 0, copy_names)
    return nothing
end
function add_aggregate_constraints(m, mode::MixedMode{T}, copy_names) where T
    if isempty(mode.function_cache)
        return nothing
    end
    eps = Dict{Int, T}()
    for list in (
        mode.constraint_mode_map_c,
        mode.constraint_mode_map_v,
        Dict(nothing=>mode.default)
    )
        for md in values(list)
            val, idx = _get_eps_idx(md)
            if haskey(eps, idx) && idx > 0
                eps[idx] = max(eps[idx], val)
            else #idx >= 0
                eps[idx] = val
            end
        end
    end
    for (idx, func) in mode.function_cache
        _add_aggregate_constraints(
            m, func, eps[idx], idx, copy_names)
    end
    return nothing
end

function _get_eps_idx(mode::ProductMode{T}) where T
    idx = mode.aggregation_group
    eps = mode.epsilon
    return eps, idx
end
function _get_eps_idx(mode)
    return 0.0, 0
end

function add_complement(mode::MixedMode{T}, m, comp::Complement,
        idxmap_primal, idxmap_dual, copy_names::Bool, pass_start::Bool) where T
    _mode = get_mode(mode, comp.constraint, idxmap_primal)
    accept_vector_set(_mode, comp)
    ret = add_complement(_mode, m, comp,
        idxmap_primal, idxmap_dual, copy_names, pass_start)
    add_function_to_cache(mode, _mode)
    return ret
end
add_function_to_cache(mode::MixedMode{T}, _mode) where T = nothing
function add_function_to_cache(mode::MixedMode{T}, _mode::ProductMode{T}) where T
    idx = _mode.aggregation_group
    if _mode.function_cache !== nothing && idx > 0
        if haskey(mode.function_cache, idx)
            mode.function_cache[idx] = MOIU.operate(+, T,
                mode.function_cache[idx], _mode.function_cache)
        else
            mode.function_cache[idx] = _mode.function_cache
        end
        _mode.function_cache = nothing
    end
    return nothing
end

function get_mode(mode::MixedMode{T}, ci::CI{F,S}, map) where {
    T,
    F<:MOI.SingleVariable,
    S<:Union{MOI.EqualTo{T}, MOI.LessThan{T}, MOI.GreaterThan{T}}
}
    # key = map[VI(ci.value)]
    key = VI(ci.value)
    if haskey(mode.constraint_mode_map_v, key)
        return mode.constraint_mode_map_v[key]
    else
        return mode.default
    end
end
function get_mode(mode::MixedMode{T}, ci::CI{F,S}, map) where {T,F,S}
    key = ci
    if haskey(mode.constraint_mode_map_c, key)
        return mode.constraint_mode_map_c[key]
    else
        return mode.default
    end
end

function add_complement(mode::ComplementMode{T}, m, comp::Complement,
    idxmap_primal, idxmap_dual, copy_names::Bool, pass_start::Bool) where T
    f = comp.func_w_cte
    s = comp.set_w_zero
    v = comp.variable
    out_var = VI[]
    out_ctr = CI[]

    with_slack = mode.with_slack

    f_dest = MOIU.map_indices.(Ref(idxmap_primal), f)

    dual = idxmap_dual[v]

    if with_slack
        slack, slack_in_set = MOI.add_constrained_variable(m, s)
        new_f = MOIU.operate(-, T, f_dest, MOI.SingleVariable(slack))
        equality = MOIU.normalize_and_add_constraint(m, new_f, MOI.EqualTo(zero(T)))

        if pass_start
            val = MOIU.eval_variables(
                x-> nothing_to_nan(MOI.get(m, MOI.VariablePrimalStart(), x)), f_dest)
            if !isnan(val)
                MOI.set(m, MOI.VariablePrimalStart(), slack, val)
            end
        end

        c = MOI.add_constraint(m, 
            MOI.VectorOfVariables([slack, dual]),
            MOI.Complements(1))
        if copy_names
            nm = MOI.get(m, MOI.VariableName(), dual)
            MOI.set(m, MOI.VariableName(), slack, "slk_($(nm))")
            MOI.set(m, MOI.ConstraintName(), slack_in_set, "bound_slk_($(nm))")
            MOI.set(m, MOI.ConstraintName(), equality, "eq_slk_($(nm))")
            MOI.set(m, MOI.ConstraintName(), c, "compl_complWslk_($(nm))")
        end

        appush!(out_var, slack)
        appush!(out_ctr, slack_in_set)
        appush!(out_ctr, equality)
        appush!(out_ctr, c)
    else
        new_f = MOIU.operate(vcat, T, f_dest, MOI.SingleVariable(dual))

        c = MOI.add_constraint(m, 
            new_f,
            MOI.Complements(1))

        if copy_names
            nm = MOI.get(m, MOI.VariableName(), dual)
            MOI.set(m, MOI.ConstraintName(), c, "compl_compl_($(nm))")
        end

        appush!(out_ctr, c)
    end

    return out_var, out_ctr
end

function add_complement(mode::SOS1Mode{T}, m, comp::Complement,
    idxmap_primal, idxmap_dual, copy_names::Bool, pass_start::Bool) where T
    f = comp.func_w_cte
    s = comp.set_w_zero
    v = comp.variable

    if comp.is_vec
        error("Vector constraint is not supported by SOS1 mode")
    end

    slack, slack_in_set = MOI.add_constrained_variable(m, s)
    f_dest = MOIU.map_indices.(Ref(idxmap_primal), f)
    new_f = MOIU.operate(-, T, f_dest, MOI.SingleVariable(slack))
    equality = MOIU.normalize_and_add_constraint(m, new_f, MOI.EqualTo(zero(T)))

    dual = idxmap_dual[v]
    c1 = MOI.add_constraint(m, 
        MOI.VectorOfVariables([slack, dual]),
        MOI.SOS1([1.0, 2.0]))

    if copy_names
        nm = MOI.get(m, MOI.VariableName(), dual)
        MOI.set(m, MOI.VariableName(), slack, "slk_($(nm))")
        MOI.set(m, MOI.ConstraintName(), slack_in_set, "bound_slk_($(nm))")
        MOI.set(m, MOI.ConstraintName(), equality, "eq_slk_($(nm))")
        MOI.set(m, MOI.ConstraintName(), c1, "compl_sos1_($(nm))")
    end

    return slack, slack_in_set, equality, c1
end

is_equality(set::S) where {S<:MOI.AbstractSet} = false
is_equality(set::MOI.EqualTo{T}) where T = true
is_equality(set::MOI.Zeros) = true

only_variable_functions(v::MOI.VariableIndex) = MOI.SingleVariable(v)
only_variable_functions(v::Vector{MOI.VariableIndex}) = MOI.VectorOfVariables(v)

nothing_to_nan(val) = ifelse(val === nothing, NaN, val)

function add_function_to_cache(mode::ProductMode{T}, func) where {T}
    if mode.function_cache === nothing
        mode.function_cache = func
    else
        mode.function_cache, func
        mode.function_cache = MOIU.operate(+, T, mode.function_cache, func)
    end
    return nothing
end
function add_complement(mode::ProductMode{T}, m, comp::Complement,
    idxmap_primal, idxmap_dual, copy_names::Bool, pass_start::Bool) where T
    f = comp.func_w_cte
    s = comp.set_w_zero
    v = comp.variable

    out_var = VI[]
    out_ctr = CI[]

    eps = mode.epsilon
    with_slack = mode.with_slack

    f_dest = MOIU.map_indices(x->idxmap_primal[x], f)

    dual = comp.is_vec ? map(x->idxmap_dual[x], v) : idxmap_dual[v]

    if with_slack
        slack, slack_in_set = if comp.is_vec
            MOI.add_constrained_variables(m, s)
        else
            MOI.add_constrained_variable(m, s)
        end
        new_f = MOIU.operate(-, T, f_dest, only_variable_functions(slack))
        if comp.is_vec
            equality = MOIU.normalize_and_add_constraint(m, new_f, MOI.Zeros(length(slack)))
        else
            equality = MOIU.normalize_and_add_constraint(m, new_f, MOI.EqualTo(zero(T)))
        end

        prod_f = MOIU.operate(dot, T, only_variable_functions(slack), only_variable_functions(dual))

        appush!(out_var, slack)
        appush!(out_ctr, slack_in_set)
        appush!(out_ctr, equality)

        if mode.aggregation_group == 0
            prod_f1 = MOIU.operate(-, T, prod_f, eps)
            c1 = MOIU.normalize_and_add_constraint(m,
                prod_f1,
                MOI.LessThan{Float64}(0.0))
            appush!(out_ctr, c1)
            if comp.is_vec
                prod_f2 = MOIU.operate(+, T, prod_f, eps)
                c2 = MOIU.normalize_and_add_constraint(m,
                    prod_f2,
                    MOI.GreaterThan{Float64}(0.0))
                appush!(out_ctr, c2)
            end
        else
            add_function_to_cache(mode, prod_f)
        end

        if pass_start
            val = MOIU.eval_variables(
                x-> nothing_to_nan(MOI.get(m, MOI.VariablePrimalStart(), x)), f_dest)
            if comp.is_vec
                for i in eachindex(val)
                    if !isnan(val[i])
                        MOI.set(m, MOI.VariablePrimalStart(), slack[i], val[i])
                    end
                end
            else
                if !isnan(val)
                    MOI.set(m, MOI.VariablePrimalStart(), slack, val)
                end
            end
        end

        if copy_names
            nm = MOI.get(m, MOI.VariableName(), dual)
            MOI.set(m, MOI.VariableName(), slack, "slk_($(nm))")
            MOI.set(m, MOI.ConstraintName(), slack_in_set, "bound_slk_($(nm))")
            MOI.set(m, MOI.ConstraintName(), equality, "eq_slk_($(nm))")
            if mode.aggregation_group == 0
                MOI.set(m, MOI.ConstraintName(), c1, "compl_prodWslk_($(nm))")
                if comp.is_vec
                    MOI.set(m, MOI.ConstraintName(), c2, "compl_prodWslk2_($(nm))")
                end
            end
        end
    else
        new_f = MOIU.operate(dot, T, f_dest, only_variable_functions(dual))
        if mode.aggregation_group == 0
            new_f1 = MOIU.operate(-, T, new_f, eps)
            c1 = MOIU.normalize_and_add_constraint(m, 
                new_f1,
                MOI.LessThan{T}(0.0))
            appush!(out_ctr, c1)
            if comp.is_vec # conic
                new_f2 = MOIU.operate(+, T, new_f, eps)
                c2 = MOIU.normalize_and_add_constraint(m, 
                    new_f2,
                    MOI.GreaterThan{T}(0.0))
                appush!(out_ctr, c2)
            end
            if copy_names
                nm = if comp.is_vec
                    MOI.get.(m, MOI.VariableName(), dual)
                else
                    MOI.get(m, MOI.VariableName(), dual)
                end
                MOI.set(m, MOI.ConstraintName(), c1, "compl_prod_($(nm))")
                if comp.is_vec
                    MOI.set(m, MOI.ConstraintName(), c2, "compl_prod2_($(nm))")
                end
            end
        else
            add_function_to_cache(mode, new_f)
        end
    end
    return out_var, out_ctr
end

function add_complement(mode::IndicatorMode{T}, m, comp::Complement,
    idxmap_primal, idxmap_dual, copy_names::Bool, pass_start::Bool) where T
    f = comp.func_w_cte
    s = comp.set_w_zero
    v = comp.variable

    method = mode.mode

    is_tight = false
    has_start = false

    f_dest = MOIU.map_indices(x->idxmap_primal[x], f)

    dual = idxmap_dual[v]

    if comp.is_vec
        error("Vector constraint is (currently) not supported by indicator mode")
    end

    if copy_names
        nm = if comp.is_vec
            MOI.get.(m, MOI.VariableName(), dual)
        else
            MOI.get(m, MOI.VariableName(), dual)
        end
    end

    vb1 = MOI.add_variable(m)
    if copy_names
        MOI.set(m, MOI.VariableName(), vb1, "compl_bin1_($(nm))")
    end

    cb1 = MOI.add_constraint(m, SVF(vb1), MOI.ZeroOne())
    if method == ONE_ONE || method == ZERO_ZERO
        # second binary
        vb2 = MOI.add_variable(m)
        cb2 = MOI.add_constraint(m, SVF(vb2), MOI.ZeroOne())
        if copy_names
            MOI.set(m, MOI.VariableName(), vb2, "compl_bin2_($(nm))")
        end

        # z1 + z2 == 1
        fb = MOIU.operate(+, T, SVF(vb1), SVF(vb2))
        cb = MOI.add_constraint(m, fb, MOI.EqualTo{T}(one(T)))
        if copy_names
            MOI.set(m, MOI.ConstraintName(), cb, "compl_sum_bin_($(nm))")
        end
    else
        vb2 = vb1
    end

    pre_f1, pre_s1 = MOIU.normalize_constant(f_dest, MOI.EqualTo(zero(T)))
    f1 = MOIU.operate(vcat, T, SVF(vb1), pre_f1)
    f2 = MOIU.operate(vcat, T, SVF(vb2), SVF(dual))

    if pass_start
        val = MOIU.eval_variables(
            x-> nothing_to_nan(MOI.get(m, MOI.VariablePrimalStart(), x)), f_dest)
        if !isnan(val)
            is_tight = abs(val) < 1e-8
            has_start = true
        end
    end

    if method == ONE_ONE
        s1 = MOI.IndicatorSet{MOI.ACTIVATE_ON_ONE}(pre_s1)
        s2 = MOI.IndicatorSet{MOI.ACTIVATE_ON_ONE}(MOI.EqualTo(zero(T)))
        if pass_start && has_start
            MOI.set(m, MOI.VariablePrimalStart(), vb1, ifelse(is_tight, 1.0, 0.0))
            MOI.set(m, MOI.VariablePrimalStart(), vb2, ifelse(is_tight, 0.0, 1.0))
        end
    elseif method == ZERO_ZERO
        s1 = MOI.IndicatorSet{MOI.ACTIVATE_ON_ZERO}(pre_s1)
        s2 = MOI.IndicatorSet{MOI.ACTIVATE_ON_ZERO}(MOI.EqualTo(zero(T)))
        if pass_start && has_start
            MOI.set(m, MOI.VariablePrimalStart(), vb1, ifelse(is_tight, 0.0, 1.0))
            MOI.set(m, MOI.VariablePrimalStart(), vb2, ifelse(is_tight, 1.0, 0.0))
        end
    else
        s1 = MOI.IndicatorSet{MOI.ACTIVATE_ON_ONE}(pre_s1)
        s2 = MOI.IndicatorSet{MOI.ACTIVATE_ON_ZERO}(MOI.EqualTo(zero(T)))
        if pass_start && has_start
            MOI.set(m, MOI.VariablePrimalStart(), vb1, ifelse(is_tight, 1.0, 0.0))
        end
    end

    # do not MOIU.normalize_and_add_constraint because are vector functions
    c1 = MOI.add_constraint(m, to_vector_affine(f1), s1)
    c2 = MOI.add_constraint(m, to_vector_affine(f2), s2)

    if copy_names
        MOI.set(m, MOI.ConstraintName(), c1, "compl_ind1_($(nm))")
        MOI.set(m, MOI.ConstraintName(), c2, "compl_ind2_($(nm))")
    end
    return c1
end

function flip_set(set::MOI.LessThan{T}) where T
    return MOI.GreaterThan{T}(0.0)
end
function flip_set(set::MOI.GreaterThan{T}) where T
    return MOI.LessThan{T}(0.0)
end

function get_bounds(var, map, fallback_bound = Inf)
    if haskey(map, var)
        info = map[var]
        # TODO deal with precision and performance
        lower = ifelse(info.lower != -Inf, info.lower, -fallback_bound)
        upper = ifelse(info.upper != +Inf, info.upper, +fallback_bound)
        return IntervalArithmetic.interval(lower, upper)
    elseif 0.0 <= fallback_bound <= Inf
        return IntervalArithmetic.interval(-fallback_bound, fallback_bound)
    else
        error("variable $var has no finite bounds defined")
    end
end

function set_bound(inter::IntervalArithmetic.Interval, ::LT{T}) where T
    return inter.hi
end
function set_bound(inter::IntervalArithmetic.Interval, ::GT{T}) where T
    return inter.lo
end

function add_complement(mode::FortunyAmatMcCarlMode{T}, m, comp::Complement,
    idxmap_primal, idxmap_dual, copy_names::Bool, pass_start::Bool) where T

    f = comp.func_w_cte
    s = comp.set_w_zero
    v = comp.variable

    is_tight = false
    has_start = false

    if mode.with_slack
        slack, slack_in_set = MOI.add_constrained_variable(m, s)
    end
    f_dest = MOIU.map_indices.(Ref(idxmap_primal), f)

    f_bounds = MOIU.eval_variables(vi -> get_bounds(vi, mode.cache.map, mode.primal_big_M), f_dest)

    if pass_start
        val = MOIU.eval_variables(
            x-> nothing_to_nan(MOI.get(m, MOI.VariablePrimalStart(), x)), f_dest)
        if !isnan(val)
            is_tight = abs(val) < 1e-8
            has_start = true
        end
    end

    if mode.with_slack
        new_f = MOIU.operate(-, T, f_dest, MOI.SingleVariable(slack))
        equality = MOIU.normalize_and_add_constraint(m, new_f, MOI.EqualTo(zero(T)))
        if pass_start && has_start
            MOI.set(m, MOI.VariablePrimalStart(), slack, val)
        end
    end

    dual = idxmap_dual[v]
    v_bounds = get_bounds(dual, mode.cache.map, mode.dual_big_M)

    bin = MOI.add_variable(m)
    if pass_start && has_start && is_tight
        MOI.set(m, MOI.VariablePrimalStart(), bin, 1.0)
        MOI.set(m, MOI.VariablePrimalStart(), dual, 0.0)
    else
        MOI.set(m, MOI.VariablePrimalStart(), bin, 0.0)
    end

    s1 = flip_set(s)
    s2 = flip_set(s)

    Ms = set_bound(f_bounds, s1)
    Mv = set_bound(v_bounds, s2)

    if isnan(Ms) || abs(Ms) >= Inf || isnan(Mv) || abs(Mv) >= Inf
        error("It was not possible to automatically compute bounds"*
            " for a complementarity pair, please add the arguments"*
            " primal_big_M and dual_big_M to FortunyAmatMcCarlMode")
    end
    
    if mode.with_slack
        f1 = MOI.ScalarAffineFunction{T}(
            MOI.ScalarAffineTerm{T}.(
                [one(T), -Ms], [slack, bin]
            ),
            0.0
        )
    else
        push!(f_dest.terms, MOI.ScalarAffineTerm{T}(-Ms, bin))
        f1 = f_dest
    end

    f2 = MOI.ScalarAffineFunction{T}(
        MOI.ScalarAffineTerm{T}.(
            [one(T), Mv], [dual, bin]
        ),
        -Mv
    )

    c1 = MOIU.normalize_and_add_constraint(m, f1, s2)
    c2 = MOIU.normalize_and_add_constraint(m, f2, s2)
    c3 = MOI.add_constraint(m, MOI.SingleVariable(bin), MOI.ZeroOne())

    if copy_names
        nm = MOI.get(m, MOI.VariableName(), dual)
        if mode.with_slack
            MOI.set(m, MOI.VariableName(), slack, "slk_($(nm))")
            MOI.set(m, MOI.ConstraintName(), slack_in_set, "bound_slk_($(nm))")
            MOI.set(m, MOI.ConstraintName(), equality, "eq_slk_($(nm))")
        end
        MOI.set(m, MOI.VariableName(), bin, "bin_($(nm))")
        MOI.set(m, MOI.ConstraintName(), c1, "compl_fa_sl_($(nm))")
        MOI.set(m, MOI.ConstraintName(), c2, "compl_fa_dl_($(nm))")
        MOI.set(m, MOI.ConstraintName(), c2, "compl_fa_bn_($(nm))")
    end

    # if mode.with_slack
    #     return slack, slack_in_set, equality, c1
    # else
    # end
    return c1
end

function to_vector_affine(f::MOI.VectorAffineFunction{T}) where T
    return f
end
function to_vector_affine(f::MOI.VectorOfVariables)
    return MOI.VectorAffineFunction{Float64}(f)
end

function handle_lower_objective_sense(lower::MOI.ModelLike)
    lower_objective_sense = MOI.get(lower, MOI.ObjectiveSense())
    if lower_objective_sense == MOI.FEASIBILITY_SENSE
        throw(ErrorException("Lower level models with objective_sense: " * 
                            lower_objective_sense * 
                            " are not supported."))
    end
    return
end