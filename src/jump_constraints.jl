function _in_upper(cref::BilevelConstraintRef)
    return cref.model.ctr_info[cref.index].level == UPPER_ONLY
end

function _in_lower(cref::BilevelConstraintRef)
    return cref.model.ctr_info[cref.index].level == LOWER_ONLY
end

function _raw_ref(model::BilevelModel, idx::Int)
    if haskey(model.ctr_upper, idx)
        return model.ctr_upper[idx]
    elseif haskey(model.ctr_lower, idx)
        return model.ctr_lower[idx]
    else
        error("Index $(idx) does not belong to BilevelModel")
    end
end

_raw_ref(cref::BilevelConstraintRef) = _raw_ref(cref.model, cref.index)

function BilevelConstraintRef(model, idx)
    raw = _raw_ref(model, idx)
    return JuMP.ConstraintRef(model, idx, raw.shape)
end

level(cref::BilevelConstraintRef) = cref.model.ctr_info[cref.index].level

function JuMP.add_constraint(
    ::BilevelModel,
    ::JuMP.AbstractConstraint,
    ::String = "",
)
    return error(
        "Can't add constraint directly to the bilevel model `m`, " *
        "attach the constraint to the upper or lower model " *
        "with @constraint(Upper(m), ...) or @constraint(Lower(m), ...)",
    )
end

function JuMP.constraint_object(con_ref::ConstraintRef{BilevelModel,Int})
    raw = _raw_ref(con_ref)
    return JuMP.constraint_object(raw)
end

# JuMP.add_constraint(m::UpperModel, c::JuMP.VectorConstraint, name::String="") =
#     error("no vec ctr")
function JuMP.add_constraint(
    m::InnerBilevelModel,
    c::Union{JuMP.ScalarConstraint{F,S},JuMP.VectorConstraint{F,S}},
    name::String = "",
) where {F,S}
    blm = bilevel_model(m)
    blm.nextconidx += 1
    cref = JuMP.ConstraintRef(blm, blm.nextconidx, JuMP.shape(c))
    func = JuMP.jump_function(c)
    level_func =
        replace_variables(func, bilevel_model(m), mylevel_var_list(m), level(m))
    level_c = JuMP.build_constraint(error, level_func, c.set)
    level_cref = JuMP.add_constraint(mylevel_model(m), level_c, name)
    mylevel_ctr_list(m)[cref.index] = level_cref
    blm.ctr_info[cref.index] = _empty_info(level(m), c)
    if !(F <: BilevelJuMP.BilevelVariableRef)
        JuMP.set_name(cref, name)
    end
    blm.ctr_upper_rev = nothing
    blm.ctr_lower_rev = nothing
    return cref
end

function JuMP.is_valid(m::BilevelModel, cref::BilevelConstraintRef)
    return cref.index in keys(m.ctr_info)
end

function JuMP.is_valid(m::InnerBilevelModel, cref::BilevelConstraintRef)
    return JuMP.is_valid(bilevel_model(m), cref) && level(cref) == level(m)
end

function JuMP.constraint_object(cref::BilevelConstraintRef, F::Type, S::Type)
    cidx = cref.index
    model = cref.model
    level = model.ctr_info[cidx].level
    if _in_upper(cref)
        con = JuMP.constraint_object(model.ctr_upper[cidx], F, S)
        return _reverse_replace_variable(con, Upper(model))
    else
        con = JuMP.constraint_object(model.ctr_lower[cidx], F, S)
        return _reverse_replace_variable(con, Lower(model))
    end
end

function _reverse_replace_variable(
    con::JuMP.VectorConstraint,
    m::InnerBilevelModel,
)
    func = _reverse_replace_variable(con.func, m)
    return JuMP.VectorConstraint(func, con.set, con.shape)
end

function _reverse_replace_variable(
    con::JuMP.ScalarConstraint,
    m::InnerBilevelModel,
)
    func = _reverse_replace_variable(con.func, m)
    return JuMP.ScalarConstraint(func, con.set)
end

function _empty_info(level, c::JuMP.ScalarConstraint{F,S}) where {F,S}
    return BilevelConstraintInfo{Float64}(level)
end

function _empty_info(level, c::JuMP.VectorConstraint{F,S}) where {F,S}
    return BilevelConstraintInfo{Vector{Float64}}(level, MOI.dimension(c.set))
end

function _assert_dim(cref, array::Vector, value::Vector)
    if length(array) != length(value)
        error(
            "For the Vector constraint {$(cref)}, expected a Vector of length = $(length(array)) and got a Vector of length = $(length(value))",
        )
    end
    return
end

function _assert_dim(cref, array::Vector, value::Number)
    error(
        "For the Vector constraint {$(cref)}, expected a Vector (of length = $(length(array))) and got the scalar $value",
    )
    return
end

function _assert_dim(cref, array::Number, value::Number)
    return
end

function _assert_dim(cref, array::Number, value::Vector)
    error(
        "For the Scalar constraint {$(cref)}, expected a Scalar and got the Vector $(value)",
    )
    return
end

function JuMP.set_dual_start_value(
    cref::BilevelConstraintRef,
    value::T,
) where {T<:Number}
    _assert_dim(cref, cref.model.ctr_info[cref.index].start, value)
    return cref.model.ctr_info[cref.index].start = value
end

function JuMP.set_dual_start_value(
    cref::BilevelConstraintRef,
    value::T,
) where {T<:Vector{S}} where {S}
    array = cref.model.ctr_info[cref.index].start
    _assert_dim(cref, array, value)
    return copyto!(array, value)
end

function JuMP.dual_start_value(cref::BilevelConstraintRef)
    return cref.model.ctr_info[cref.index].start
end

"""
    set_dual_upper_bound_hint(cref, value)

Set a upper bound to the dual variable of the constraint `cref` to `value`.
This bound will not be dualized.
The dual upper bound hint is used to help the solution method.

Solution `mode`s can be benefitted from this hint:

* `BigMMode` will use this information to compute a tighter bound for the
  dual variable.

* Other modes will be stabilized by the existence of the bounds on variables
  that would otherwise no be bounded.

* Bounds that are not dualized are also useful for binary expansions of
  products of variables that can be done with `QuadraticToBinary.jl`.
"""
function set_dual_upper_bound_hint(
    cref::BilevelConstraintRef,
    value::T,
) where {T<:Number}
    _assert_dim(cref, cref.model.ctr_info[cref.index].upper, value)
    return cref.model.ctr_info[cref.index].upper = value
end

function set_dual_upper_bound_hint(
    cref::BilevelConstraintRef,
    value::T,
) where {T<:Vector{S}} where {S}
    array = cref.model.ctr_info[cref.index].upper
    _assert_dim(cref, array, value)
    return copyto!(array, value)
end


"""
    get_dual_upper_bound_hint(cref)

Get the upper bound to the dual variable of the constraint `cref` that was
set with `set_dual_upper_bound_hint`.
"""
function get_dual_upper_bound_hint(cref::BilevelConstraintRef)
    return cref.model.ctr_info[cref.index].upper
end

"""
    set_dual_lower_bound_hint(cref, value)

Set a lower bound to the dual variable of the constraint `cref` to `value`.
This bound will not be dualized.
The dual lower bound hint is used to help the solution method.

Solution `mode`s can be benefitted from this hint:

* `BigMMode` will use this information to compute a tighter bound for the
  dual variable.

* Other modes will be stabilized by the existence of the bounds on variables
  that would otherwise no be bounded.

* Bounds that are not dualized are also useful for binary expansions of
  products of variables that can be done with `QuadraticToBinary.jl`.
"""
function set_dual_lower_bound_hint(
    cref::BilevelConstraintRef,
    value::T,
) where {T<:Number}
    _assert_dim(cref, cref.model.ctr_info[cref.index].lower, value)
    return cref.model.ctr_info[cref.index].lower = value
end

function set_dual_lower_bound_hint(
    cref::BilevelConstraintRef,
    value::T,
) where {T<:Vector{S}} where {S}
    array = cref.model.ctr_info[cref.index].lower
    _assert_dim(cref, array, value)
    return copyto!(array, value)
end

"""
    get_dual_lower_bound_hint(cref)

Get the lower bound to the dual variable of the constraint `cref` that was
set with `set_dual_lower_bound_hint`.
"""
function get_dual_lower_bound_hint(cref::BilevelConstraintRef)
    return cref.model.ctr_info[cref.index].lower
end

"""
    set_primal_upper_bound_hint(vref, value)

Set a upper bound to the prima variable `vref` to `value`.
This bound will not be dualized.
The upper bound hint is used to help the solution method.

Solution `mode`s can be benefitted from this hint:

* `BigMMode` will use this information to compute a tighter bound for the
  primal constraint variable.

* Other modes will be stabilized by the existence of the bounds on variables
  that would otherwise no be bounded.

* Bounds that are not dualized are also useful for binary expansions of
  products of variables that can be done with `QuadraticToBinary.jl`.
"""
function set_primal_upper_bound_hint(
    vref::BilevelVariableRef,
    value::T,
) where {T<:Number}
    return vref.model.var_info[vref.idx].upper = value
end

"""
    get_primal_upper_bound_hint(cref)

Get the upper bound to the primal variable of the constraint `cref` that was
set with `set_primal_upper_bound_hint`.
"""
function get_primal_upper_bound_hint(vref::BilevelVariableRef)
    return vref.model.var_info[vref.idx].upper
end

"""
    set_primal_lower_bound_hint(vref, value)

Set a lower bound to the prima variable `vref` to `value`.
This bound will not be dualized.
The lower bound hint is used to help the solution method.

Solution `mode`s can be benefitted from this hint:

* `BigMMode` will use this information to compute a tighter bound for the
  primal constraint variable.

* Other modes will be stabilized by the existence of the bounds on variables
  that would otherwise no be bounded.

* Bounds that are not dualized are also useful for binary expansions of
  products of variables that can be done with `QuadraticToBinary.jl`.
"""
function set_primal_lower_bound_hint(
    vref::BilevelVariableRef,
    value::T,
) where {T<:Number}
    return vref.model.var_info[vref.idx].lower = value
end

"""
    get_primal_lower_bound_hint(cref)

Get the lower bound to the primal variable of the constraint `cref` that was
set with `set_primal_lower_bound_hint`.
"""
function get_primal_lower_bound_hint(vref::BilevelVariableRef)
    return vref.model.var_info[vref.idx].lower
end

function JuMP.value(cref::BilevelConstraintRef; result::Int = 1)
    if _in_lower(cref)
        # Constraint index on the lower model
        con_lower_idx = cref.model.ctr_lower[cref.index].index
        # Single bilevel model constraint associated with the lower level constraint
        con_sblm_idx = cref.model.lower_to_sblm[con_lower_idx]
    else
        # Constraint index on the lower model
        con_upper_idx = cref.model.ctr_upper[cref.index].index
        # Single bilevel model constraint associated with the lower level constraint
        con_sblm_idx = cref.model.upper_to_sblm[con_upper_idx]
    end
    # Solver constraint associated with the single bilevel model constraint
    con_solver_idx = cref.model.sblm_to_solver[con_sblm_idx]
    return MOI.get(
        cref.model.solver,
        MOI.ConstraintPrimal(result),
        con_solver_idx,
    )
end
# variables again (duals)
# code for using dual variables associated with lower level constraints
# in the upper level

function JuMP.num_constraints(model::BilevelModel)
    return length(model.ctr_info)
end
function JuMP.num_constraints(model::LowerModel)
    return length(model.m.ctr_lower)
end
function JuMP.num_constraints(model::UpperModel)
    return length(model.m.ctr_upper)
end
function JuMP.num_constraints(model::InnerBilevelModel, f, s)
    return JuMP.num_constraints(mylevel_model(model), f, s)
end
function JuMP.num_constraints(model::BilevelModel, f, s)
    return JuMP.num_constraints(Upper(model), f, s) +
           JuMP.num_constraints(Lower(model), f, s)
end

"""
    DualOf(constraint::ConstraintRef)

Get the dual variable associated with a constraint. This is only valid
for constraints in the upper level of a bilevel model.

## Examples

```jldoctest
julia> m = BilevelModel();

julia> @variable(Lower(m), x >= 0);

julia> @constraint(Lower(m), c, x <= 1);

julia> @variable(Upper(m), y, DualOf(c));
```
"""
struct DualOf
    ci::BilevelConstraintRef
end

function DualOf(::AbstractArray{<:T}) where {T<:JuMP.ConstraintRef}
    return error(
        "If you are trying to do something like:\n" *
        "@constraint(Lower(m), my_constraint_vector[t in 1:T], ...)\n" *
        "@variable(Upper(m), my_variable[1:N], " *
        "DualOf(my_constraint_vector))\n" *
        "Either do:\n" *
        "@variable(Upper(m), my_variable[t=1:N], " *
        "DualOf(my_constraint_vector[t]))\n" *
        "Or use anonynous variables:\n" *
        "@variable(Upper(m), variable_type = DualOf(my_constraint_vector[t]))",
    )
end

struct DualVariableInfo
    info::JuMP.VariableInfo
    ci::BilevelConstraintRef
end

function JuMP.build_variable(
    _error::Function,
    info::JuMP.VariableInfo,
    dual_of::DualOf;
    extra_kw_args...,
)
    if level(dual_of.ci) != LOWER_ONLY
        error(
            "Variables can only be tied to LOWER level constraints, got $(dual_of.ci.level) level",
        )
    end
    for (kwarg, _) in extra_kw_args
        _error("Unrecognized keyword argument $kwarg")
    end

    if info.has_lb
        set_dual_lower_bound_hint(dual_of.ci, info.lower_bound)
        # info.has_lb = false
        # info.lower_bound = NaN
    end
    if info.has_ub
        set_dual_upper_bound_hint(dual_of.ci, info.upper_bound)
        # info.has_ub = false
        # info.upper_bound = NaN
    end
    info.has_fix && _error("Dual variable does not support fixing")
    if info.has_start
        JuMP.set_dual_start_value(dual_of.ci, info.start)
        # info.has_start = false
        # info.start = NaN
    end
    info.binary && _error("Dual variable cannot be binary")
    info.integer && _error("Dual variable cannot be integer")

    info = JuMP.VariableInfo(
        false,
        NaN,
        false,
        NaN,
        false,
        NaN,
        false,
        NaN,
        false,
        false,
    )

    return DualVariableInfo(info, dual_of.ci)
end
function JuMP.add_variable(
    inner::UpperModel,
    dual_info::DualVariableInfo,
    name::String = "",
)
    # TODO vector version
    m = bilevel_model(inner)
    m.last_variable_index += 1
    vref = BilevelVariableRef(m, m.last_variable_index, DUAL_OF_LOWER)
    v_upper =
        JuMP.add_variable(m.upper, JuMP.ScalarVariable(dual_info.info), name)
    m.var_upper[vref.idx] = v_upper
    m.upper_var_to_lower_ctr_link[v_upper] = m.ctr_lower[dual_info.ci.index]
    m.var_info[vref.idx] = _empty_info(DUAL_OF_LOWER)
    JuMP.set_name(vref, name)
    m.var_upper_rev = nothing
    m.var_lower_rev = nothing
    return vref
end

function get_constrain_ref(vref::BilevelVariableRef)
    model = vref.model
    ctr_ref = model.upper_var_to_lower_ctr_link[model.var_upper[vref.idx]]
    idx = -1
    for (ind, ref) in model.ctr_lower
        if ref == ctr_ref
            idx = ind
        end
    end
    @assert idx != -1
    return BilevelConstraintRef(model, idx)
end

function JuMP.dual(cref::BilevelConstraintRef)
    # Right now this code assumes there is no possibility for vectorized constraints
    if _in_lower(cref)
        # Constraint index on the lower model
        con_lower_ref = cref.model.ctr_lower[cref.index]
        con_lower_idx = con_lower_ref.index
        # Dual variable associated with constraint index
        model_var_idxs =
            cref.model.lower_primal_dual_map.primal_con_dual_var[con_lower_idx]
        # Single bilevel model variable associated with the dual variable
        sblm_var_idxs = MOI.VariableIndex[]
        for vi in model_var_idxs
            push!(sblm_var_idxs, cref.model.lower_dual_to_sblm[vi])
        end
        # Solver variable associated withe the sblm model
        solver_var_idxs = MOI.VariableIndex[]
        for vi in sblm_var_idxs
            push!(solver_var_idxs, cref.model.sblm_to_solver[vi])
        end
        pre_duals =
            MOI.get(cref.model.solver, MOI.VariablePrimal(), solver_var_idxs)
        return JuMP.reshape_vector(
            pre_duals,
            JuMP.dual_shape(con_lower_ref.shape),
        )
    elseif _in_upper(cref)
        m = cref.model
        con_upper_ref = cref.model.ctr_upper[cref.index]
        solver_ctr_idx =
            m.sblm_to_solver[m.upper_to_sblm[JuMP.index(con_upper_ref)]]
        pre_duals =
            MOI.get(cref.model.solver, MOI.ConstraintDual(), solver_ctr_idx)
        return JuMP.reshape_vector(
            pre_duals,
            JuMP.dual_shape(con_upper_ref.shape),
        )
    else
        error(
            "Dual solutions of upper level constraints are not available. Either the solution method does nto porvide duals or or the solver failed to get one.",
        )
    end
end

function JuMP.normalized_rhs(cref::BilevelConstraintRef)
    return JuMP.normalized_rhs(_raw_ref(cref))
end

function JuMP.set_normalized_rhs(cref::BilevelConstraintRef, val)
    return JuMP.set_normalized_rhs(_raw_ref(cref), val)
end

function JuMP.add_to_function_constant(cref::BilevelConstraintRef, val)
    return JuMP.add_to_function_constant(_raw_ref(cref), val)
end

function JuMP.normalized_coefficient(
    cref::BilevelConstraintRef,
    var::BilevelVariableRef,
)
    model = cref.model
    level_var = if _in_upper(cref)
        model.var_upper[var.idx]
    else
        model.var_lower[var.idx]
    end
    return JuMP.normalized_coefficient(_raw_ref(cref), level_var)
end

function JuMP.set_normalized_coefficient(
    cref::BilevelConstraintRef,
    var::BilevelVariableRef,
    val,
)
    model = cref.model
    level_var = if _in_upper(cref)
        model.var_upper[var.idx]
    else
        model.var_lower[var.idx]
    end
    return JuMP.set_normalized_coefficient(_raw_ref(cref), level_var, val)
end

function JuMP.list_of_constraint_types(
    model::InnerBilevelModel,
)::Vector{Tuple{DataType,DataType}}
    return JuMP.list_of_constraint_types(mylevel_model(model))
end
function JuMP.list_of_constraint_types(
    model::BilevelModel,
)::Vector{Tuple{DataType,DataType}}
    return unique!(
        vcat(
            JuMP.list_of_constraint_types(Upper(model)),
            JuMP.list_of_constraint_types(Lower(model)),
        ),
    )
end

function JuMP.all_constraints(model::BilevelModel, f, s)
    return unique!(
        vcat(
            JuMP.all_constraints(Upper(model), f, s),
            JuMP.all_constraints(Lower(model), f, s),
        ),
    )
end

function JuMP.all_constraints(model::InnerBilevelModel, f, s)
    _build_reverse_ctr_map!(model)
    m = mylevel_model(model)
    list = JuMP.all_constraints(m, f, s)
    return _get_reverse_ctr_map.(model, list)
end
function _build_reverse_ctr_map!(um::UpperModel)
    m = bilevel_model(um)
    m.ctr_upper_rev = Dict{JuMP.ConstraintRef,JuMP.ConstraintRef}()
    for (idx, ref) in m.ctr_upper
        m.ctr_upper_rev[ref] = BilevelConstraintRef(m, idx)
    end
end
function _build_reverse_ctr_map!(lm::LowerModel)
    m = bilevel_model(lm)
    m.ctr_lower_rev = Dict{JuMP.ConstraintRef,JuMP.ConstraintRef}()
    for (idx, ref) in m.ctr_lower
        m.ctr_lower_rev[ref] = BilevelConstraintRef(m, idx)
    end
    return nothing
end
_get_reverse_ctr_map(m::UpperModel, idx) = m.m.ctr_upper_rev[idx]
_get_reverse_ctr_map(m::LowerModel, idx) = m.m.ctr_lower_rev[idx]

function JuMP.delete(mod::BilevelModel, cref::BilevelConstraintRef)
    model = cref.model
    @assert model === mod
    idx = cref.index
    if haskey(model.ctr_upper, idx)
        c_up = model.ctr_upper[idx]
        delete!(model.ctr_upper, idx)
        JuMP.delete(model.upper, c_up)
    end
    if haskey(model.ctr_lower, idx)
        c_lo = model.ctr_lower[idx]
        delete!(model.ctr_lower, idx)
        JuMP.delete(model.lower, c_lo)
    end
    delete!(model.ctr_info, idx)
    model.ctr_upper_rev = nothing
    model.ctr_lower_rev = nothing
    return nothing
end
