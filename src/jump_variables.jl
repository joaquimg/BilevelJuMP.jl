
mylevel(v::BilevelVariableRef) = v.level
function in_level(v::BilevelVariableRef, level::Level)
    return (
        v.level === LOWER_BOTH ||
        v.level === UPPER_BOTH ||
        v.level === level ||
        (v.level === DUAL_OF_LOWER && level === UPPER_ONLY)
    )
end

in_level(v::BilevelVariableRef, ::UpperModel) = _in_upper(v)
in_level(v::BilevelVariableRef, ::LowerModel) = _in_lower(v)
_in_upper(v::BilevelVariableRef) = _in_upper(mylevel(v))
_in_lower(v::BilevelVariableRef) = _in_lower(mylevel(v))
upper_ref(v::BilevelVariableRef) = v.model.var_upper[v.idx]
lower_ref(v::BilevelVariableRef) = v.model.var_lower[v.idx]

"""
    BilevelVariableRef

Alias for `GenericAffExpr{Float64,BilevelVariableRef}`.
"""
const BilevelAffExpr = GenericAffExpr{Float64,BilevelVariableRef}

"""
    BilevelQuadExpr

Alias for `GenericQuadExpr{Float64,BilevelVariableRef}`.
"""
const BilevelQuadExpr = GenericQuadExpr{Float64,BilevelVariableRef}

function jump_var_ref(v::BilevelVariableRef)
    level = mylevel(v)
    if level == LOWER_ONLY || level == LOWER_BOTH
        return lower_ref(v)
    else
        return upper_ref(v)
    end
end

function solver_ref(v::BilevelVariableRef)
    m = v.model
    if mylevel(v) == LOWER_ONLY
        return m.sblm_to_solver[m.lower_to_sblm[JuMP.index(lower_ref(v))]]
    else
        return m.sblm_to_solver[m.upper_to_sblm[JuMP.index(upper_ref(v))]]
    end
end

Base.broadcastable(v::BilevelVariableRef) = Ref(v)
Base.copy(v::BilevelVariableRef) = v
function Base.:(==)(v::BilevelVariableRef, w::BilevelVariableRef)
    return v.model === w.model && v.idx == w.idx && v.level == w.level
end
JuMP.owner_model(v::BilevelVariableRef) = v.model
JuMP.isequal_canonical(v::BilevelVariableRef, w::BilevelVariableRef) = v == w
# add in both levels
function JuMP.add_variable(
    inner::InnerBilevelModel,
    v::JuMP.AbstractVariable,
    name::String = "",
)
    m = bilevel_model(inner)
    m.last_variable_index += 1
    vref = BilevelVariableRef(m, m.last_variable_index, level_both(inner))
    # break info so that bounds go to correct level
    var_upper, var_lower = split_variable(inner, v)
    v_upper = JuMP.add_variable(m.upper, var_upper, name)
    m.var_upper[vref.idx] = v_upper
    v_lower = JuMP.add_variable(m.lower, var_lower, name)
    m.var_lower[vref.idx] = v_lower
    set_link!(inner, v_upper, v_lower)
    m.var_info[vref.idx] = _empty_info(level_both(inner))
    JuMP.set_name(vref, name)
    m.var_upper_rev = nothing
    m.var_lower_rev = nothing
    return vref
end
function JuMP.add_variable(
    single::SingleBilevelModel,
    v::JuMP.AbstractVariable,
    name::String = "",
)
    m = bilevel_model(single)
    m.last_variable_index += 1
    vref = BilevelVariableRef(m, m.last_variable_index, level(single))
    v_level = JuMP.add_variable(mylevel_model(single), v, name)
    mylevel_var_list(single)[vref.idx] = v_level
    push_single_level_variable!(single, v_level)
    m.var_info[vref.idx] = _empty_info(level(single))
    JuMP.set_name(vref, name)
    m.var_upper_rev = nothing
    m.var_lower_rev = nothing
    return vref
end
function JuMP.delete(::BilevelModel, vref::BilevelVariableRef)
    model = vref.model
    idx = vref.idx
    delete!(model.var_info, idx)
    if haskey(model.var_upper, idx)
        v_up = model.var_upper[idx]
        delete!(model.var_upper, idx)
        delete!(model.upper_only, v_up)
        delete!(model.upper_to_lower_link, v_up)
        delete!(model.upper_var_to_lower_ctr_link, v_up)
        delete!(model.link, v_up)
        JuMP.delete(model.upper, v_up)
    end
    if haskey(model.var_lower, idx)
        v_lo = model.var_lower[idx]
        delete!(model.var_lower, idx)
        delete!(model.lower_only, v_lo)
        delete!(model.lower_to_upper_link, v_lo)
        JuMP.delete(model.lower, v_lo)
    end
    model.var_upper_rev = nothing
    model.var_lower_rev = nothing
    return nothing
end
function JuMP.is_valid(m::BilevelModel, vref::BilevelVariableRef)
    return vref.idx in keys(m.var_info)
end
function JuMP.is_valid(m::InnerBilevelModel, vref::BilevelVariableRef)
    return JuMP.is_valid(bilevel_model(m), vref) && in_level(vref, level(m))
end
JuMP.num_variables(m::BilevelModel) = length(m.var_info)
JuMP.num_variables(m::UpperModel) = length(m.m.var_upper)
JuMP.num_variables(m::LowerModel) = length(m.m.var_lower)
function _empty_info(level::Level)
    return BilevelVariableInfo(level)
end

function JuMP.all_variables(m::InnerBilevelModel)
    list = keys(mylevel_var_list(m))
    model = bilevel_model(m)
    ret = BilevelVariableRef[]
    for idx in list
        var = BilevelVariableRef(model, idx)
        if in_level(var, m)
            push!(ret, var)
        end
    end
    return ret
end
function JuMP.all_variables(model::BilevelModel)
    vec = JuMP.all_variables(Upper(model))
    append!(vec, JuMP.all_variables(Lower(model)))
    return unique(vec)
end

"""
Split variable because actual owner of the variable should be the one holding the bounds.
"""
function split_variable(::UpperModel, v::JuMP.AbstractVariable)
    var_upper = v
    var_lower = JuMP.ScalarVariable(
        JuMP.VariableInfo(
            false,
            NaN,
            false,
            NaN,
            false,
            NaN,
            v.info.has_start,
            v.info.start,
            false,
            false,
        ),
    )
    return var_upper, var_lower
end
function split_variable(::LowerModel, v::JuMP.AbstractVariable)
    var_lower = v
    var_upper = JuMP.ScalarVariable(
        JuMP.VariableInfo(
            false,
            NaN,
            false,
            NaN,
            false,
            NaN,
            v.info.has_start,
            v.info.start,
            false,
            false,
        ),
    )
    return var_upper, var_lower
end

function JuMP.has_lower_bound(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        return !isnan(get_dual_lower_bound_hint(get_constrain_ref(vref)))
    end
    return JuMP.has_lower_bound(jump_var_ref(vref))
end
function JuMP.lower_bound(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        return get_dual_lower_bound_hint(get_constrain_ref(vref))
    end
    # @assert !JuMP.is_fixed(vref)
    return JuMP.lower_bound(jump_var_ref(vref))
end
function JuMP.set_lower_bound(vref::BilevelVariableRef, lower::Number)
    if mylevel(vref) == DUAL_OF_LOWER
        set_dual_lower_bound_hint(get_constrain_ref(vref), lower)
        return
    end
    JuMP.set_lower_bound(jump_var_ref(vref), lower)
    return
end
function JuMP.delete_lower_bound(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        set_dual_lower_bound_hint(get_constrain_ref(vref), NaN)
        return
    end
    JuMP.delete_lower_bound(jump_var_ref(vref))
    return
end
function JuMP.has_upper_bound(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        return !isnan(get_dual_upper_bound_hint(get_constrain_ref(vref)))
    end
    return JuMP.has_upper_bound(jump_var_ref(vref))
end
function JuMP.upper_bound(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        return get_dual_upper_bound_hint(get_constrain_ref(vref))
    end
    # @assert !JuMP.is_fixed(vref)
    return JuMP.upper_bound(jump_var_ref(vref))
end
function JuMP.set_upper_bound(vref::BilevelVariableRef, upper)
    if mylevel(vref) == DUAL_OF_LOWER
        set_dual_upper_bound_hint(get_constrain_ref(vref), upper)
        return
    end
    JuMP.set_upper_bound(jump_var_ref(vref), upper)
    return
end
function JuMP.delete_upper_bound(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        set_dual_upper_bound_hint(get_constrain_ref(vref), NaN)
        return
    end
    return JuMP.delete_upper_bound(jump_var_ref(vref))
end

JuMP.is_fixed(vref::BilevelVariableRef) = JuMP.is_fixed(jump_var_ref(vref))
JuMP.fix_value(vref::BilevelVariableRef) = JuMP.fix_value(jump_var_ref(vref))
function JuMP.fix(vref::BilevelVariableRef, value; force::Bool = false)
    if mylevel(vref) == DUAL_OF_LOWER
        error("Dual variable cannot be fixed.")
    end
    return JuMP.fix(jump_var_ref(vref), value; force = force)
end
function JuMP.unfix(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        error("Dual variable cannot be fixed.")
    end
    return JuMP.unfix(jump_var_ref(vref))
end

function JuMP.start_value(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        return JuMP.dual_start_value(get_constrain_ref(vref))
    end
    return JuMP.start_value(jump_var_ref(vref))
end
function JuMP.set_start_value(vref::BilevelVariableRef, start)
    if mylevel(vref) == DUAL_OF_LOWER
        JuMP.set_dual_start_value(get_constrain_ref(vref), start)
        return
    end
    _in_upper(vref) && JuMP.set_start_value(upper_ref(vref), start)
    _in_lower(vref) && JuMP.set_start_value(lower_ref(vref), start)
    return
end

JuMP.is_binary(vref::BilevelVariableRef) = JuMP.is_binary(jump_var_ref(vref))
function JuMP.set_binary(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        error("Dual variable cannot be binary.")
    end
    return JuMP.set_binary(jump_var_ref(vref))
end
function JuMP.unset_binary(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        error("Dual variable cannot be binary.")
    end
    return JuMP.unset_binary(jump_var_ref(vref))
end
JuMP.is_integer(vref::BilevelVariableRef) = JuMP.is_integer(jump_var_ref(vref))
function JuMP.set_integer(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        error("Dual variable cannot be integer.")
    end
    return JuMP.set_integer(jump_var_ref(vref))
end
function JuMP.unset_integer(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        error("Dual variable cannot be integer.")
    end
    return JuMP.unset_integer(jump_var_ref(vref))
end

function JuMP.value(v::BilevelVariableRef; result::Int = 1)::Float64
    m = owner_model(v)
    solver = m.solver
    ref = solver_ref(v)
    return MOI.get(solver, MOI.VariablePrimal(result), ref)
end
