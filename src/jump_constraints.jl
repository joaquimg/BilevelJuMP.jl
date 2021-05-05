function raw_ref(model::BilevelModel, idx::Int)
    if haskey(model.ctr_upper, idx)
        return model.ctr_upper[idx]
    elseif haskey(model.ctr_lower, idx)
        return model.ctr_lower[idx]
    else
        error("Index $(idx) does not belong to BilevelModel")
    end
end
raw_ref(cref::BilevelConstraintRef) = raw_ref(cref.model, cref.index)

function BilevelConstraintRef(model, idx)
    raw = raw_ref(model, idx)
    JuMP.ConstraintRef(model, idx, raw.shape)
end
JuMP.constraint_type(::AbstractBilevelModel) = BilevelConstraintRef
level(cref::BilevelConstraintRef) = cref.model.ctr_level[cref.index]
function JuMP.add_constraint(::BilevelModel, ::JuMP.AbstractConstraint, ::String="")
    error(
        "Can't add constraint directly to the bilevel model `m`, "*
        "attach the constraint to the upper or lower model "*
        "with @constraint(Upper(m), ...) or @constraint(Lower(m), ...)")
end
function JuMP.constraint_object(con_ref::ConstraintRef{BilevelModel, Int})
    raw = raw_ref(con_ref)
    return JuMP.constraint_object(raw)
end
# JuMP.add_constraint(m::UpperModel, c::JuMP.VectorConstraint, name::String="") =
#     error("no vec ctr")
function JuMP.add_constraint(m::InnerBilevelModel, c::Union{JuMP.ScalarConstraint{F,S},JuMP.VectorConstraint{F,S}}, name::String="") where {F,S}
    blm = bilevel_model(m)
    blm.nextconidx += 1
    cref = JuMP.ConstraintRef(blm, blm.nextconidx, JuMP.shape(c))
    func = JuMP.jump_function(c)
    level_func = replace_variables(func, bilevel_model(m), mylevel_model(m), mylevel_var_list(m), level(m))
    level_c = JuMP.build_constraint(error, level_func, c.set)
    level_cref = JuMP.add_constraint(mylevel_model(m), level_c, name)
    blm.ctr_level[cref.index] = level(m)
    mylevel_ctr_list(m)[cref.index] = level_cref
    blm.constraints[cref.index] = c
    blm.ctr_info[cref.index] = empty_info(c)
    JuMP.set_name(cref, name)
    cref
end
function JuMP.delete(m::AbstractBilevelModel, cref::BilevelConstraintRef)
    error("can't delete")
    # m.need_rebuild = true
    # delete!(m.constraints, cref.index)
    # delete!(m.connames, cref.index)
end
JuMP.is_valid(m::BilevelModel, cref::BilevelConstraintRef) = cref.index in keys(m.constraints)
JuMP.is_valid(m::InnerBilevelModel, cref::BilevelConstraintRef) =
    JuMP.is_valid(bilevel_model(m), cref) && level(cref) == level(m)
function JuMP.constraint_object(cref::BilevelConstraintRef, F::Type, S::Type)
    c = cref.model.constraints[cref.index]
    # `TypeError` should be thrown is `F` and `S` are not correct
    # This is needed for the tests in `constraints.jl`
    c.func::F
    c.set::S
    c
end
function empty_info(c::JuMP.ScalarConstraint{F,S}) where {F,S}
    return ConstraintInfo{Float64}()
end
function empty_info(c::JuMP.VectorConstraint{F,S}) where {F,S}
    return ConstraintInfo{Vector{Float64}}(MOI.dimension(c.set))
end

function JuMP.set_dual_start_value(cref::BilevelConstraintRef, value::T) where T<:Number
    cref.model.ctr_info[cref.index].start = value
end
function JuMP.set_dual_start_value(cref::BilevelConstraintRef, value::T) where T<:Vector{S} where S
    array = cref.model.ctr_info[cref.index].start
    @assert length(array) == length(value)
    copyto!(array, value)
end
function JuMP.dual_start_value(cref::BilevelConstraintRef)
    cref.model.ctr_info[cref.index].start
end
function set_dual_upper_bound(cref::BilevelConstraintRef, value::T) where T<:Number
    cref.model.ctr_info[cref.index].upper = value
end
function set_dual_upper_bound(cref::BilevelConstraintRef, value::T) where T<:Vector{S} where S
    array = cref.model.ctr_info[cref.index].upper
    @assert length(array) == length(value)
    copyto!(array, value)
end
function get_dual_upper_bound(cref::BilevelConstraintRef)
    cref.model.ctr_info[cref.index].upper
end
function set_dual_lower_bound(cref::BilevelConstraintRef, value::T) where T<:Number
    cref.model.ctr_info[cref.index].lower = value
end
function set_dual_lower_bound(cref::BilevelConstraintRef, value::T) where T<:Vector{S} where S
    array = cref.model.ctr_info[cref.index].lower
    @assert length(array) == length(value)
    copyto!(array, value)
end
function get_dual_lower_bound(cref::BilevelConstraintRef)
    cref.model.ctr_info[cref.index].lower
end

function set_primal_upper_bound_hint(vref::BilevelVariableRef, value::T) where T<:Number
    vref.model.var_info[vref.idx].upper = value
end
function get_primal_upper_bound_hint(vref::BilevelVariableRef)
    vref.model.var_info[vref.idx].upper
end
function set_primal_lower_bound_hint(vref::BilevelVariableRef, value::T) where T<:Number
    vref.model.var_info[vref.idx].lower = value
end
function get_primal_lower_bound_hint(vref::BilevelVariableRef)
    vref.model.var_info[vref.idx].lower
end

function JuMP.value(cref::BilevelConstraintRef; result::Int = 1)
    if level(cref) == BilevelJuMP.LOWER_ONLY
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
    return MOI.get(cref.model.solver, MOI.ConstraintPrimal(result), con_solver_idx)
end
# variables again (duals)
# code for using dual variables associated with lower level constraints
# in the upper level

function JuMP.num_constraints(model::BilevelModel)
    return length(model.ctr_level)
end
function JuMP.num_constraints(model::LowerModel)
    return length(model.m.ctr_lower)
end
function JuMP.num_constraints(model::UpperModel)
    return length(model.m.ctr_upper)
end

struct DualOf
    ci::BilevelConstraintRef
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
        error("Variables can only be tied to LOWER level constraints, got $(dual_of.ci.level) level")
    end
    for (kwarg, _) in extra_kw_args
        _error("Unrecognized keyword argument $kwarg")
    end

    if info.has_lb
        set_dual_lower_bound(dual_of.ci, info.lower_bound)
        # info.has_lb = false
        # info.lower_bound = NaN
    end
    if info.has_ub
        set_dual_upper_bound(dual_of.ci, info.upper_bound)
        # info.has_ub = false
        # info.upper_bound = NaN
    end
    info.has_fix   && _error("Dual variable does not support fixing")
    if info.has_start
        JuMP.set_dual_start_value(dual_of.ci, info.start)
        # info.has_start = false
        # info.start = NaN
    end
    info.binary    && _error("Dual variable cannot be binary")
    info.integer   && _error("Dual variable cannot be integer")

    info = JuMP.VariableInfo(false, NaN, false, NaN,
                             false, NaN, false, NaN,
                             false, false)

    return DualVariableInfo(
        info,
        dual_of.ci
    )
end
function JuMP.add_variable(inner::UpperModel, dual_info::DualVariableInfo, name::String="")
    # TODO vector version
    m = bilevel_model(inner)
    m.nextvaridx += 1
    vref = BilevelVariableRef(m, m.nextvaridx, DUAL_OF_LOWER)
    v_upper = JuMP.add_variable(m.upper, JuMP.ScalarVariable(dual_info.info), name)
    m.var_upper[vref.idx] = v_upper
    m.var_level[vref.idx] = DUAL_OF_LOWER
    m.upper_var_to_lower_ctr_link[v_upper] = m.ctr_lower[dual_info.ci.index] # TODO improve this
    m.variables[vref.idx] = JuMP.ScalarVariable(dual_info.info)
    JuMP.set_name(vref, name)
    m.var_info[vref.idx] = empty_info(dual_info)
    vref
end
function empty_info(::DualVariableInfo)
    return VariableInfo()
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
    if level(cref) == BilevelJuMP.LOWER_ONLY
        # Constraint index on the lower model
        con_lower_ref = cref.model.ctr_lower[cref.index]
        con_lower_idx = con_lower_ref.index
        # Dual variable associated with constraint index
        model_var_idxs = cref.model.lower_primal_dual_map.primal_con_dual_var[con_lower_idx]
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
        pre_duals = MOI.get(cref.model.solver, MOI.VariablePrimal(), solver_var_idxs)
        return JuMP.reshape_vector(
            pre_duals,
            JuMP.dual_shape(con_lower_ref.shape)
            )
    elseif level(cref) == BilevelJuMP.UPPER_ONLY
        m = cref.model
        con_upper_ref = cref.model.ctr_upper[cref.index]
        solver_ctr_idx = m.sblm_to_solver[m.upper_to_sblm[JuMP.index(con_upper_ref)]]
        pre_duals = MOI.get(cref.model.solver, MOI.ConstraintDual(), solver_ctr_idx)
        return JuMP.reshape_vector(
            pre_duals,
            JuMP.dual_shape(con_upper_ref.shape)
            )
    else
        error("Dual solutions of upper level constraints are not available. Either the solution method does nto porvide duals or or the solver failed to get one.")
    end
end

function JuMP.normalized_rhs(cref::BilevelConstraintRef)
    return JuMP.normalized_rhs(raw_ref(cref))
end

function JuMP.set_normalized_rhs(cref::BilevelConstraintRef, val)
    return JuMP.set_normalized_rhs(raw_ref(cref), val)
end

function JuMP.add_to_function_constant(cref::BilevelConstraintRef, val)
    return JuMP.add_to_function_constant(raw_ref(cref), val)
end

function JuMP.normalized_coefficient(cref::BilevelConstraintRef, var::BilevelVariableRef)
    cidx = cref.index
    model = cref.model
    level = model.ctr_level[cidx]
    vidx = var.idx
    level_var = if level == UPPER_ONLY
        model.var_upper[vidx]
    else
        model.var_lower[vidx]
    end
    return JuMP.normalized_coefficient(raw_ref(cref), level_var)
end

function JuMP.set_normalized_coefficient(
    cref::BilevelConstraintRef, var::BilevelVariableRef, val)
    cidx = cref.index
    model = cref.model
    level = model.ctr_level[cidx]
    vidx = var.idx
    level_var = if level == UPPER_ONLY
        model.var_upper[vidx]
    else
        model.var_lower[vidx]
    end
    return JuMP.set_normalized_coefficient(raw_ref(cref), level_var, val)
end