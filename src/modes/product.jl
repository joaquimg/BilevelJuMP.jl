"""
    ProductMode(epsilon = 0.0; with_slack = false, aggregation_group = nothing)

Used to solve a bilevel problem with the
MPEC reformulation using products  to convert complementarity constraints
into non-convex quadratic constraints.

* `with_slack` indicates whether to use slack variables to reformulate the
  complementarity constraints. Given a pair `expr` and `var`, the reformulation
  is `expr == slack` and `var * slack == 0` instead of `expr * slack == 0`.

* `aggregation_group` indicates whether to aggregate the products into a single
  quadratic constraint. If `aggregation_group` is `nothing`, then each product
  is converted into a quadratic constraint. If `aggregation_group` is a positive
  integer, then products with the same `aggregation_group` are aggregated into
  a single quadratic constraint.
"""
mutable struct ProductMode{T} <: AbstractBilevelSolverMode{T}
    epsilon::T
    with_slack::Bool
    aggregation_group::Int # only useful in mixed mode
    function_cache::Union{Nothing,MOI.AbstractScalarFunction}
    function ProductMode(
        eps::T = zero(Float64);
        with_slack::Bool = false,
        aggregation_group = nothing,
    ) where {T<:Float64} # Real
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

function reset!(mode::ProductMode)
    mode.function_cache = nothing
    return nothing
end

accept_vector_set(::ProductMode{T}, ::Complement) where {T} = nothing

function add_complement(
    mode::ProductMode{T},
    m,
    comp::Complement,
    idxmap_primal,
    idxmap_dual,
    copy_names::Bool,
    pass_start::Bool,
) where {T}
    f = comp.func_w_cte
    s = comp.set_w_zero
    v = comp.variable

    out_var = VI[]
    out_ctr = CI[]

    eps = mode.epsilon
    with_slack = mode.with_slack

    f_dest = MOIU.map_indices(x -> idxmap_primal[x], f)

    dual = comp.is_vec ? map(x -> idxmap_dual[x], v) : idxmap_dual[v]

    if with_slack
        slack, slack_in_set = if comp.is_vec
            MOI.add_constrained_variables(m, s)
        else
            MOI.add_constrained_variable(m, s)
        end
        new_f = MOIU.operate(-, T, f_dest, _only_variable_functions(slack))
        if comp.is_vec
            equality = MOIU.normalize_and_add_constraint(
                m,
                new_f,
                MOI.Zeros(length(slack)),
            )
        else
            equality = MOIU.normalize_and_add_constraint(
                m,
                new_f,
                MOI.EqualTo(zero(T)),
            )
        end

        prod_f = MOIU.operate(
            LinearAlgebra.dot,
            T,
            _only_variable_functions(slack),
            _only_variable_functions(dual),
        )

        _appush!(out_var, slack)
        _appush!(out_ctr, slack_in_set)
        _appush!(out_ctr, equality)

        if mode.aggregation_group == 0
            prod_f1 = MOIU.operate(-, T, prod_f, eps)
            c1 = MOIU.normalize_and_add_constraint(
                m,
                prod_f1,
                MOI.LessThan{Float64}(0.0),
            )
            _appush!(out_ctr, c1)
            if comp.is_vec
                prod_f2 = MOIU.operate(+, T, prod_f, eps)
                c2 = MOIU.normalize_and_add_constraint(
                    m,
                    prod_f2,
                    MOI.GreaterThan{Float64}(0.0),
                )
                _appush!(out_ctr, c2)
            end
        else
            add_function_to_cache(mode, prod_f)
        end

        if pass_start
            val = MOIU.eval_variables(
                x -> nothing_to_nan(MOI.get(m, MOI.VariablePrimalStart(), x)),
                f_dest,
            )
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
            # MOI.set(m, MOI.ConstraintName(), slack_in_set, "bound_slk_($(nm))")
            MOI.set(m, MOI.ConstraintName(), equality, "eq_slk_($(nm))")
            if mode.aggregation_group == 0
                MOI.set(m, MOI.ConstraintName(), c1, "compl_prodWslk_($(nm))")
                if comp.is_vec
                    MOI.set(
                        m,
                        MOI.ConstraintName(),
                        c2,
                        "compl_prodWslk2_($(nm))",
                    )
                end
            end
        end
    else
        new_f = MOIU.operate(
            LinearAlgebra.dot,
            T,
            f_dest,
            _only_variable_functions(dual))
        if mode.aggregation_group == 0
            new_f1 = MOIU.operate(-, T, new_f, eps)
            c1 = MOIU.normalize_and_add_constraint(
                m,
                new_f1,
                MOI.LessThan{T}(0.0),
            )
            _appush!(out_ctr, c1)
            if comp.is_vec # conic
                new_f2 = MOIU.operate(+, T, new_f, eps)
                c2 = MOIU.normalize_and_add_constraint(
                    m,
                    new_f2,
                    MOI.GreaterThan{T}(0.0),
                )
                _appush!(out_ctr, c2)
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

function add_aggregate_constraints(m, mode::ProductMode, copy_names)
    if mode.function_cache === nothing
        return nothing
    end
    _add_aggregate_constraints(
        m,
        mode.function_cache,
        mode.epsilon,
        0,
        copy_names,
    )
    return nothing
end

function add_function_to_cache(mode::ProductMode{T}, func) where {T}
    if mode.function_cache === nothing
        mode.function_cache = func
    else
        mode.function_cache, func
        mode.function_cache = MOIU.operate(+, T, mode.function_cache, func)
    end
    return nothing
end

_only_variable_functions(v::MOI.VariableIndex) = v
_only_variable_functions(v::Vector{MOI.VariableIndex}) = MOI.VectorOfVariables(v)
