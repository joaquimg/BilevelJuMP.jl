"""
    IndicatorSetting

The type of indicator function to use in the `IndicatorMode` mode.
"""
@enum IndicatorSetting ZERO_ONE ZERO_ZERO ONE_ONE

@doc(
    "Activates the indicator constraint on the primal constraint if the " *
    "auxiliaty binary is zero " *
    "and activates the indicator constraint on the dual variable if the " *
    "auxiliary binary is one.",
    ZERO_ONE
)

@doc(
    "Activates the indicator constraint on the primal constraint if the " *
    "auxiliaty binary is zero " *
    "and activates the indicator constraint on the dual variable if the " *
    "auxiliary binary is zero.",
    ZERO_ZERO
)

@doc(
    "Activates the indicator constraint on the primal constraint if the " *
    "auxiliaty binary is one " *
    "and activates the indicator constraint on the dual variable if the " *
    "auxiliary binary is one.",
    ONE_ONE
)

"""
    IndicatorMode(method::IndicatorSetting = BilevelJuMP.ONE_ONE)

Used to solve a bilevel problem with the
MPEC reformulation using indicator constaints to convert complementarity
constraints to a mixed integer formulation.

* `method` indicates how the indicator constraints are activated for primal
  cosntraints and dual variables. See `IndicatorSetting` for more details.
"""
mutable struct IndicatorMode{T} <: AbstractBilevelSolverMode{T}
    method::IndicatorSetting
    function IndicatorMode(method::IndicatorSetting = ONE_ONE)
        return new{Float64}(method)
    end
end

function add_complement(
    mode::IndicatorMode{T},
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

    method = mode.method

    is_tight = false
    has_start = false

    f_dest = MOIU.map_indices(x -> idxmap_primal[x], f)

    dual = idxmap_dual[v]

    if comp.is_vec
        error(
            "Vector constraint is (currently) not supported by indicator mode",
        )
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

    cb1 = MOI.add_constraint(m, vb1, MOI.ZeroOne())
    if method == ONE_ONE || method == ZERO_ZERO
        # second binary
        vb2 = MOI.add_variable(m)
        cb2 = MOI.add_constraint(m, vb2, MOI.ZeroOne())
        if copy_names
            MOI.set(m, MOI.VariableName(), vb2, "compl_bin2_($(nm))")
        end

        # z1 + z2 == 1
        fb = MOIU.operate(+, T, vb1, vb2)
        cb = MOI.add_constraint(m, fb, MOI.EqualTo{T}(one(T)))
        if copy_names
            MOI.set(m, MOI.ConstraintName(), cb, "compl_sum_bin_($(nm))")
        end
    else
        vb2 = vb1
    end

    pre_f1, pre_s1 = MOIU.normalize_constant(f_dest, MOI.EqualTo(zero(T)))
    f1 = MOIU.operate(vcat, T, vb1, pre_f1)
    f2 = MOIU.operate(vcat, T, vb2, dual)

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

    if method == ONE_ONE
        s1 = MOI.Indicator{MOI.ACTIVATE_ON_ONE}(pre_s1)
        s2 = MOI.Indicator{MOI.ACTIVATE_ON_ONE}(MOI.EqualTo(zero(T)))
        if pass_start && has_start
            MOI.set(
                m,
                MOI.VariablePrimalStart(),
                vb1,
                ifelse(is_tight, 1.0, 0.0),
            )
            MOI.set(
                m,
                MOI.VariablePrimalStart(),
                vb2,
                ifelse(is_tight, 0.0, 1.0),
            )
        end
    elseif method == ZERO_ZERO
        s1 = MOI.Indicator{MOI.ACTIVATE_ON_ZERO}(pre_s1)
        s2 = MOI.Indicator{MOI.ACTIVATE_ON_ZERO}(MOI.EqualTo(zero(T)))
        if pass_start && has_start
            MOI.set(
                m,
                MOI.VariablePrimalStart(),
                vb1,
                ifelse(is_tight, 0.0, 1.0),
            )
            MOI.set(
                m,
                MOI.VariablePrimalStart(),
                vb2,
                ifelse(is_tight, 1.0, 0.0),
            )
        end
    else
        s1 = MOI.Indicator{MOI.ACTIVATE_ON_ONE}(pre_s1)
        s2 = MOI.Indicator{MOI.ACTIVATE_ON_ZERO}(MOI.EqualTo(zero(T)))
        if pass_start && has_start
            MOI.set(
                m,
                MOI.VariablePrimalStart(),
                vb1,
                ifelse(is_tight, 1.0, 0.0),
            )
        end
    end

    # do not MOIU.normalize_and_add_constraint because are vector functions
    c1 = MOI.add_constraint(m, _to_vector_affine(f1), s1)
    c2 = MOI.add_constraint(m, _to_vector_affine(f2), s2)

    if copy_names
        MOI.set(m, MOI.ConstraintName(), c1, "compl_ind1_($(nm))")
        MOI.set(m, MOI.ConstraintName(), c2, "compl_ind2_($(nm))")
    end
    return c1
end

function _to_vector_affine(f::MOI.VectorAffineFunction{T}) where {T}
    return f
end
function _to_vector_affine(f::MOI.VectorOfVariables)
    return MOI.VectorAffineFunction{Float64}(f)
end
