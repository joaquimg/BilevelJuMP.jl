
mylevel(v::BilevelVariableRef) = v.level
in_level(v::BilevelVariableRef, level::Level) = (
    v.level === BOTH ||
    v.level === level ||
    (v.level === DUAL_OF_LOWER && level === UPPER_ONLY))

in_upper(v::BilevelVariableRef) = in_upper(mylevel(v))
in_lower(v::BilevelVariableRef) = in_lower(mylevel(v))
upper_ref(v::BilevelVariableRef) = v.model.var_upper[v.idx]
lower_ref(v::BilevelVariableRef) = v.model.var_lower[v.idx]

function bound_ref(v::BilevelVariableRef)
    if mylevel(v) == LOWER_ONLY
        return lower_ref(v)
    elseif mylevel(v) == UPPER_ONLY || mylevel(v) == DUAL_OF_LOWER
        return upper_ref(v)
    elseif mylevel(v) == BOTH
        m = v.model
        i = v.idx
        upper_ref = m.var_upper[i]
        # this checks actual owner:
        if haskey(m.upper_to_lower_link, upper_ref)
            return upper_ref
        else
            return m.var_lower[i]
        end
    else
        error("Unknown level")
    end
end

function solver_ref(v::BilevelVariableRef)
    m = v.model
    if mylevel(v) == LOWER_ONLY
        return m.sblm_to_solver[
            m.lower_to_sblm[JuMP.index(lower_ref(v))]]
    else
        return m.sblm_to_solver[
            m.upper_to_sblm[JuMP.index(upper_ref(v))]]
    end
end

Base.broadcastable(v::BilevelVariableRef) = Ref(v)
Base.copy(v::BilevelVariableRef) = v
Base.:(==)(v::BilevelVariableRef, w::BilevelVariableRef) =
    v.model === w.model && v.idx == w.idx && v.level == w.level
JuMP.owner_model(v::BilevelVariableRef) = v.model
JuMP.isequal_canonical(v::BilevelVariableRef, w::BilevelVariableRef) = v == w
JuMP.variable_type(::AbstractBilevelModel) = BilevelVariableRef
# add in BOTH levels
function JuMP.add_variable(inner::InnerBilevelModel, v::JuMP.AbstractVariable, name::String="")
    m = bilevel_model(inner)
    m.nextvaridx += 1
    vref = BilevelVariableRef(m, m.nextvaridx, BOTH)
    # break info so that bounds go to correct level
    var_upper, var_lower = split_variable(inner, v)
    v_upper = JuMP.add_variable(m.upper, var_upper, name)
    m.var_upper[vref.idx] = v_upper
    v_lower = JuMP.add_variable(m.lower, var_lower, name)
    m.var_lower[vref.idx] = v_lower
    m.var_level[vref.idx] = BOTH
    set_link!(inner, v_upper, v_lower)
    m.variables[vref.idx] = v # save complete data
    JuMP.set_name(vref, name)
    m.var_info[vref.idx] = empty_info(v)
    vref
end
function JuMP.add_variable(single::SingleBilevelModel, v::JuMP.AbstractVariable, name::String="")
    m = bilevel_model(single)
    m.nextvaridx += 1
    vref = BilevelVariableRef(m, m.nextvaridx, level(single))
    v_level = JuMP.add_variable(mylevel_model(single), v, name)
    mylevel_var_list(single)[vref.idx] = v_level
    m.var_level[vref.idx] = level(single)
    m.variables[vref.idx] = v
    JuMP.set_name(vref, name)
    m.var_info[vref.idx] = empty_info(v)
    vref
end
function MOI.delete!(m::AbstractBilevelModel, vref::BilevelVariableRef)
    error("No deletion on bilevel models")
    delete!(m.variables, vref.idx)
    delete!(m.varnames, vref.idx)
end
MOI.is_valid(m::BilevelModel, vref::BilevelVariableRef) = vref.idx in keys(m.variables)
JuMP.num_variables(m::BilevelModel) = length(m.variables)
JuMP.num_variables(m::UpperModel) = length(m.m.var_upper)
JuMP.num_variables(m::LowerModel) = length(m.m.var_lower)
function empty_info(::JuMP.AbstractVariable)
    return VariableInfo{Float64}()
end

"""
Split variable because actual owner of the variable should be the one holding the bounds.
"""
function split_variable(::UpperModel, v::JuMP.AbstractVariable)
    var_upper = v
    var_lower = JuMP.ScalarVariable(JuMP.VariableInfo(
        false, NaN,
        false, NaN,
        false, NaN,
        v.info.has_start, v.info.start,
        false, false))
    return var_upper, var_lower
end
function split_variable(::LowerModel, v::JuMP.AbstractVariable)
    var_lower = v
    var_upper = JuMP.ScalarVariable(JuMP.VariableInfo(
        false, NaN,
        false, NaN,
        false, NaN,
        v.info.has_start, v.info.start,
        false, false))
    return var_upper, var_lower
end

# Internal function
variable_info(vref::BilevelVariableRef) = vref.model.variables[vref.idx].info
function update_variable_info(vref::BilevelVariableRef, info::JuMP.VariableInfo)
    vref.model.variables[vref.idx] = JuMP.ScalarVariable(info)
end

function JuMP.has_lower_bound(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        return !isnan(get_dual_lower_bound(get_constrain_ref(vref)))
    end
    return variable_info(vref).has_lb
end
function JuMP.lower_bound(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        return get_dual_lower_bound(get_constrain_ref(vref))
    end
    @assert !JuMP.is_fixed(vref)
    variable_info(vref).lower_bound
end
function JuMP.set_lower_bound(vref::BilevelVariableRef, lower)
    if mylevel(vref) == DUAL_OF_LOWER
        set_dual_lower_bound(get_constrain_ref(vref), lower)
        return vref.model.variables[vref.idx]
    end
    info = variable_info(vref)
    JuMP.set_lower_bound(bound_ref(vref), lower)
    update_variable_info(vref,
                         JuMP.VariableInfo(true, lower,
                                           info.has_ub, info.upper_bound,
                                           info.has_fix, info.fixed_value,
                                           info.has_start, info.start,
                                           info.binary, info.integer))
end
function JuMP.delete_lower_bound(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        set_dual_lower_bound(get_constrain_ref(vref), NaN)
        return vref.model.variables[vref.idx]
    end
    info = variable_info(vref)
    JuMP.delete_lower_bound(bound_ref(vref))
    update_variable_info(vref,
                         JuMP.VariableInfo(false, info.lower_bound,
                                           info.has_ub, info.upper_bound,
                                           info.has_fix, info.fixed_value,
                                           info.has_start, info.start,
                                           info.binary, info.integer))
end
function JuMP.has_upper_bound(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        return !isnan(get_dual_upper_bound(get_constrain_ref(vref)))
    end
    return variable_info(vref).has_ub
end
function JuMP.upper_bound(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        return get_dual_upper_bound(get_constrain_ref(vref))
    end
    @assert !JuMP.is_fixed(vref)
    variable_info(vref).upper_bound
end
function JuMP.set_upper_bound(vref::BilevelVariableRef, upper)
    if mylevel(vref) == DUAL_OF_LOWER
        set_dual_upper_bound(get_constrain_ref(vref), upper)
        return vref.model.variables[vref.idx]
    end
    info = variable_info(vref)
    JuMP.set_upper_bound(bound_ref(vref), upper)
    update_variable_info(vref,
                         JuMP.VariableInfo(info.has_lb, info.lower_bound,
                                           true, upper,
                                           info.has_fix, info.fixed_value,
                                           info.has_start, info.start,
                                           info.binary, info.integer))
end
function JuMP.delete_upper_bound(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        set_dual_upper_bound(get_constrain_ref(vref), NaN)
        return vref.model.variables[vref.idx]
    end
    info = variable_info(vref)
    JuMP.delete_upper_bound(bound_ref(vref))
    update_variable_info(vref,
                         JuMP.VariableInfo(info.has_lb, info.lower_bound,
                                           false, info.upper_bound,
                                           info.has_fix, info.fixed_value,
                                           info.has_start, info.start,
                                           info.binary, info.integer))
end
JuMP.is_fixed(vref::BilevelVariableRef) = variable_info(vref).has_fix
JuMP.fix_value(vref::BilevelVariableRef) = variable_info(vref).fixed_value
function JuMP.fix(vref::BilevelVariableRef, value; force::Bool=false)
    if mylevel(vref) == DUAL_OF_LOWER
        error("Dual variable cannot be fixed.")
    end
    info = variable_info(vref)
    JuMP.fix(bound_ref(vref), value; force=force)
    update_variable_info(vref,
                         JuMP.VariableInfo(info.has_lb, info.lower_bound,
                                           info.has_ub, info.upper_bound,
                                           true, value,
                                           info.has_start, info.start,
                                           info.binary, info.integer))
end
function JuMP.unfix(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        error("Dual variable cannot be fixed.")
    end
    info = variable_info(vref)
    JuMP.unfix(bound_ref(vref))
    update_variable_info(vref,
                         JuMP.VariableInfo(info.has_lb, info.lower_bound,
                                           info.has_ub, info.upper_bound,
                                           false, info.fixed_value,
                                           info.has_start, info.start,
                                           info.binary, info.integer))
end
function JuMP.start_value(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        return get_dual_start(get_constrain_ref(vref))
    end
    variable_info(vref).start
end
function JuMP.set_start_value(vref::BilevelVariableRef, start)
    if mylevel(vref) == DUAL_OF_LOWER
        set_dual_start(get_constrain_ref(vref), start)
        return vref.model.variables[vref.idx]
    end
    info = variable_info(vref)
    in_upper(vref) && JuMP.set_start_value(upper_ref(vref), start)
    in_lower(vref) && JuMP.set_start_value(lower_ref(vref), start)
    update_variable_info(vref,
                         JuMP.VariableInfo(info.has_lb, info.lower_bound,
                                           info.has_ub, info.upper_bound,
                                           info.has_fix, info.fixed_value,
                                           true, start,
                                           info.binary, info.integer))
end
JuMP.is_binary(vref::BilevelVariableRef) = variable_info(vref).binary
function JuMP.set_binary(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        error("Dual variable cannot be binary.")
    end
    @assert !JuMP.is_integer(vref)
    info = variable_info(vref)
    JuMP.set_binary(bound_ref(vref))
    update_variable_info(vref,
                         JuMP.VariableInfo(info.has_lb, info.lower_bound,
                                           info.has_ub, info.upper_bound,
                                           info.has_fix, info.fixed_value,
                                           info.has_start, info.start,
                                           true, info.integer))
end
function JuMP.unset_binary(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        error("Dual variable cannot be binary.")
    end
    info = variable_info(vref)
    JuMP.unset_binary(bound_ref(vref))
    update_variable_info(vref,
                         JuMP.VariableInfo(info.has_lb, info.lower_bound,
                                           info.has_ub, info.upper_bound,
                                           info.has_fix, info.fixed_value,
                                           info.has_start, info.start,
                                           false, info.integer))
end
JuMP.is_integer(vref::BilevelVariableRef) = variable_info(vref).integer
function JuMP.set_integer(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        error("Dual variable cannot be integer.")
    end
    @assert !JuMP.is_binary(vref)
    info = variable_info(vref)
    JuMP.set_integer(bound_ref(vref))
    update_variable_info(vref,
                         JuMP.VariableInfo(info.has_lb, info.lower_bound,
                                           info.has_ub, info.upper_bound,
                                           info.has_fix, info.fixed_value,
                                           info.has_start, info.start,
                                           info.binary, true))
end
function JuMP.unset_integer(vref::BilevelVariableRef)
    if mylevel(vref) == DUAL_OF_LOWER
        error("Dual variable cannot be integer.")
    end
    info = variable_info(vref)
    JuMP.unset_integer(bound_ref(vref))
    update_variable_info(vref,
                         JuMP.VariableInfo(info.has_lb, info.lower_bound,
                                           info.has_ub, info.upper_bound,
                                           info.has_fix, info.fixed_value,
                                           info.has_start, info.start,
                                           info.binary, false))
end

function JuMP.value(v::BilevelVariableRef; result::Int = 1)::Float64
    m = owner_model(v)
    solver = m.solver
    ref = solver_ref(v)
    return MOI.get(solver, MOI.VariablePrimal(result), ref)
end