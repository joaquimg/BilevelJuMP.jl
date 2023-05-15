
"""
    BigMMode(; with_slack = false, primal_big_M = Inf, dual_big_M = Inf)

Used to solve a bilevel problem with the
MPEC reformulation using Fortuny-Amat and McCarl's big-M method to convert
complementarity constraints to a mixed integer formulation.

* `with_slack` indicates whether to use slack variables to convert the
  complementarity constraints to a mixed integer formulation. If `false`, the
  reformulation of a constraint like `expr <= 0`
  is `expr <= big_M * (1 - binary)` and `var <= big_M * binary`, where `var`
  is the associated dual variable. If `true`, the reformulation is
  `expr == slack`, `slack <= big_M * (1 - binary)` and `var <= big_M * binary`.

* `primal_big_M` is a big-M used to primal variables that have no bounds so
  we can compute the big-M for the primal constraint.

* `dual_big_M` is a big-M used to dual variables that have no bounds so
  we can compute the big-M for the dual constraint.

Also known as `FortunyAmatMcCarlMode` (which can be used interchangeably).
"""
mutable struct BigMMode{T} <: AbstractBilevelSolverMode{T}
    with_slack::Bool
    primal_big_M::Float64
    dual_big_M::Float64
    cache::ComplementBoundCache
    function BigMMode(;
        with_slack = false,
        primal_big_M = Inf,
        dual_big_M = Inf,
    )
        return new{Float64}(
            with_slack,
            primal_big_M,
            dual_big_M,
            ComplementBoundCache(),
        )
    end
end

"""
    FortunyAmatMcCarlMode

See `BigMMode` for more details.
"""
const FortunyAmatMcCarlMode = BigMMode

function reset!(mode::FortunyAmatMcCarlMode)
    mode.cache = ComplementBoundCache()
    return nothing
end

function build_full_map!(
    mode::FortunyAmatMcCarlMode,
    upper_idxmap,
    lower_idxmap,
    lower_dual_idxmap,
    lower_primal_dual_map,
)
    _build_bound_map!(
        mode.cache,
        upper_idxmap,
        lower_idxmap,
        lower_dual_idxmap,
        lower_primal_dual_map,
    )
    return nothing
end

function add_complement(
    mode::FortunyAmatMcCarlMode{T},
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

    is_tight = false
    has_start = false

    if mode.with_slack
        slack, slack_in_set = MOI.add_constrained_variable(m, s)
    end
    f_dest = MOIU.map_indices.(Ref(idxmap_primal), f)

    f_bounds = MOIU.eval_variables(
        vi -> get_bounds(vi, mode.cache.map, mode.primal_big_M),
        f_dest,
    )

    if pass_start
        val = MOIU.eval_variables(
            x -> nothing_to_nan(MOI.get(m, MOI.VariablePrimalStart(), x)),
            f_dest,
        )
        if !isnan(val)
            is_tight = abs(val) < 1e-8
            has_start = true
        end
    end

    if mode.with_slack
        new_f = MOIU.operate(-, T, f_dest, slack)
        equality =
            MOIU.normalize_and_add_constraint(m, new_f, MOI.EqualTo(zero(T)))
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
        error(
            "It was not possible to automatically compute bounds" *
            " for a complementarity pair, please add the arguments" *
            " primal_big_M and dual_big_M to FortunyAmatMcCarlMode",
        )
    end

    if mode.with_slack
        f1 = MOI.ScalarAffineFunction{T}(
            MOI.ScalarAffineTerm{T}.([one(T), -Ms], [slack, bin]),
            0.0,
        )
    else
        push!(f_dest.terms, MOI.ScalarAffineTerm{T}(-Ms, bin))
        f1 = f_dest
    end

    f2 = MOI.ScalarAffineFunction{T}(
        MOI.ScalarAffineTerm{T}.([one(T), Mv], [dual, bin]),
        -Mv,
    )

    c1 = MOIU.normalize_and_add_constraint(m, f1, s2)
    c2 = MOIU.normalize_and_add_constraint(m, f2, s2)
    c3 = MOI.add_constraint(m, bin, MOI.ZeroOne())

    if copy_names
        nm = MOI.get(m, MOI.VariableName(), dual)
        if mode.with_slack
            MOI.set(m, MOI.VariableName(), slack, "slk_($(nm))")
            # MOI.set(m, MOI.ConstraintName(), slack_in_set, "bound_slk_($(nm))")
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

function flip_set(set::MOI.LessThan{T}) where {T}
    return MOI.GreaterThan{T}(0.0)
end

function flip_set(set::MOI.GreaterThan{T}) where {T}
    return MOI.LessThan{T}(0.0)
end

function get_bounds(var, map, fallback_bound = Inf)
    if haskey(map, var)
        info = map[var]
        # TODO deal with precision and performance
        lower = ifelse(info.lower != -Inf, info.lower, -fallback_bound)
        upper = ifelse(info.upper != +Inf, info.upper, +fallback_bound)
        return Interval(lower, upper)
    elseif 0.0 <= fallback_bound <= Inf
        return Interval(-fallback_bound, fallback_bound)
    else
        error("variable $var has no finite bounds defined")
    end
end

function set_bound(inter::Interval, ::LT{T}) where {T}
    return inter.hi
end

function set_bound(inter::Interval, ::GT{T}) where {T}
    return inter.lo
end
