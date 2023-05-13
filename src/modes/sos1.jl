"""
    SOS1Mode()

Used to solve a bilevel problem with the
MPEC reformulation using SOS1 constraints to convert complementarity constraints
into mixed-integer constraints.
"""
mutable struct SOS1Mode{T} <: AbstractBilevelSolverMode{T}
    function SOS1Mode()
        return new{Float64}()
    end
end

function add_complement(
    mode::SOS1Mode{T},
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

    if comp.is_vec
        error("Vector constraint is not supported by SOS1 mode")
    end

    slack, slack_in_set = MOI.add_constrained_variable(m, s)
    f_dest = MOIU.map_indices.(Ref(idxmap_primal), f)
    new_f = MOIU.operate(-, T, f_dest, slack)
    equality = MOIU.normalize_and_add_constraint(m, new_f, MOI.EqualTo(zero(T)))

    dual = idxmap_dual[v]
    c1 = MOI.add_constraint(
        m,
        MOI.VectorOfVariables([slack, dual]),
        MOI.SOS1([1.0, 2.0]),
    )

    if copy_names
        nm = MOI.get(m, MOI.VariableName(), dual)
        MOI.set(m, MOI.VariableName(), slack, "slk_($(nm))")
        # MOI.set(m, MOI.ConstraintName(), slack_in_set, "bound_slk_($(nm))")
        MOI.set(m, MOI.ConstraintName(), equality, "eq_slk_($(nm))")
        MOI.set(m, MOI.ConstraintName(), c1, "compl_sos1_($(nm))")
    end

    return slack, slack_in_set, equality, c1
end