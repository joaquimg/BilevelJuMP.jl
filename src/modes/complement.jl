"""
    ComplementMode(; with_slack = false)

Used to solve a bilevel problem with the
MPEC reformulation using actual complementarity constraints.
A limited number of solvers support this mode.
One example is Knitro.

* `with_slack` indicates whether to use slack variables to reformulate the
  complementarity constraints. Given a pair `expr` and `var`, the reformulation
  is `expr == slack` and `var ⟂ slack` instead of `expr ⟂ slack`.
"""
mutable struct ComplementMode{T} <: AbstractBilevelSolverMode{T}
    with_slack::Bool
    function ComplementMode(; with_slack = false)
        return new{Float64}(with_slack)
    end
end

function add_complement(
    mode::ComplementMode{T},
    m,
    comp::Complement,
    idxmap_primal,
    idxmap_dual,
    copy_names,
    pass_start::Bool,
) where {T}
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
        new_f = MOIU.operate(-, T, f_dest, slack)
        equality =
            MOIU.normalize_and_add_constraint(m, new_f, MOI.EqualTo(zero(T)))

        if pass_start
            val = MOIU.eval_variables(
                x -> nothing_to_nan(MOI.get(m, MOI.VariablePrimalStart(), x)),
                f_dest,
            )
            if !isnan(val)
                MOI.set(m, MOI.VariablePrimalStart(), slack, val)
            end
        end

        c = MOI.add_constraint(
            m,
            MOI.VectorOfVariables([slack, dual]),
            MOI.Complements(1),
        )
        if copy_names
            nm = MOI.get(m, MOI.VariableName(), dual)
            MOI.set(m, MOI.VariableName(), slack, "slk_($(nm))")
            # MOI.set(m, MOI.ConstraintName(), slack_in_set, "bound_slk_($(nm))")
            MOI.set(m, MOI.ConstraintName(), equality, "eq_slk_($(nm))")
            MOI.set(m, MOI.ConstraintName(), c, "compl_complWslk_($(nm))")
        end

        _appush!(out_var, slack)
        _appush!(out_ctr, slack_in_set)
        _appush!(out_ctr, equality)
        _appush!(out_ctr, c)
    else
        new_f = MOIU.operate(vcat, T, f_dest, dual)

        c = MOI.add_constraint(m, new_f, MOI.Complements(1))

        if copy_names
            nm = MOI.get(m, MOI.VariableName(), dual)
            MOI.set(m, MOI.ConstraintName(), c, "compl_compl_($(nm))")
        end

        _appush!(out_ctr, c)
    end

    return out_var, out_ctr
end