"""
    StrongDualityMode(eps = 0.0, inequality = true)

A mode that adds a strong duality constraint of the lower level problem
instead of reformulating the complementarity constraints.

* `eps`: The tolerance for the strong duality constraint. Defaults to `0.0`.

* `inequality`: If `true` the strong duality constraint is added as two
  inequality constraints. If `false` the strong duality constraint is added as
  an equality constraint. Defaults to `true`.
"""
mutable struct StrongDualityMode{T} <: AbstractBilevelSolverMode{T}
    inequality::Bool
    epsilon::T
    function StrongDualityMode(
        eps::T = zero(Float64);
        inequality = true,
    ) where {T}
        return new{T}(inequality, eps)
    end
end

ignore_dual_objective(::StrongDualityMode{T}) where {T} = false

function add_strong_duality(
    mode::StrongDualityMode{T},
    m,
    primal_obj,
    dual_obj,
    idxmap_primal,
    idxmap_dual,
) where {T}
    primal = MOIU.map_indices.(Ref(idxmap_primal), primal_obj)
    dual = MOIU.map_indices.(Ref(idxmap_dual), dual_obj)

    func = MOIU.operate(-, T, primal, dual)

    if !mode.inequality
        c = MOIU.normalize_and_add_constraint(m, func, MOI.EqualTo(zero(T)))
        MOI.set(m, MOI.ConstraintName(), c, "lower_strong_duality")
        return CI[c]
    else
        func_up = MOIU.operate(-, T, func, mode.epsilon)
        c_up =
            MOIU.normalize_and_add_constraint(m, func_up, MOI.LessThan(zero(T)))
        MOI.set(m, MOI.ConstraintName(), c_up, "lower_strong_duality_up")

        func_lo = MOIU.operate(+, T, func, mode.epsilon)
        c_lo = MOIU.normalize_and_add_constraint(
            m,
            func_lo,
            MOI.GreaterThan(zero(T)),
        )
        MOI.set(m, MOI.ConstraintName(), c_lo, "lower_strong_duality_lo")

        return CI[c_up, c_lo]
    end
end