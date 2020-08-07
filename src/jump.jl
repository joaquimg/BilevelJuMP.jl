
# The following is largely inspired from JuMP/test/JuMPExtension.jl

@enum Level BOTH LOWER_ONLY UPPER_ONLY DUAL_OF_LOWER

abstract type AbstractBilevelModel <: JuMP.AbstractModel end

Base.broadcastable(model::AbstractBilevelModel) = Ref(model)

mutable struct BilevelModel <: AbstractBilevelModel
    # Structured data
    upper::JuMP.AbstractModel
    lower::JuMP.AbstractModel

    # Model data
    nextvaridx::Int                                 # Next variable index is nextvaridx+1
    variables::Dict{Int, JuMP.AbstractVariable}     # Map varidx -> variable
    varnames::Dict{Int, String}                     # Map varidx -> name
    varnames_rev::Dict{String, Int}                 # Map varidx -> name
    var_level::Dict{Int, Level}
    var_upper::Dict{Int, JuMP.AbstractVariableRef}
    var_lower::Dict{Int, JuMP.AbstractVariableRef} #
    var_info::Dict{Int, VariableInfo}

    # upper level decisions that are "parameters" of the second level
    upper_to_lower_link::Dict{JuMP.AbstractVariableRef, JuMP.AbstractVariableRef}
    # lower level decisions that are input to upper
    lower_to_upper_link::Dict{JuMP.AbstractVariableRef, JuMP.AbstractVariableRef}
    # lower level decisions that are input to upper
    upper_var_to_lower_ctr_link::Dict{JuMP.AbstractVariableRef, JuMP.ConstraintRef}
    # joint link
    link::Dict{JuMP.AbstractVariableRef, JuMP.AbstractVariableRef}

    nextconidx::Int                                 # Next constraint index is nextconidx+1
    constraints::Dict{Int, JuMP.AbstractConstraint} # Map conidx -> variable
    connames::Dict{Int, String}                     # Map varidx -> name
    connames_rev::Dict{String, Int}                     # Map varidx -> name
    ctr_level::Dict{Int, Level}
    ctr_upper::Dict{Int, JuMP.ConstraintRef}
    ctr_lower::Dict{Int, JuMP.ConstraintRef}
    ctr_info::Dict{Int, ConstraintInfo}

    upper_objective_sense::MOI.OptimizationSense
    upper_objective_function::JuMP.AbstractJuMPScalar

    lower_objective_sense::MOI.OptimizationSense
    lower_objective_function::JuMP.AbstractJuMPScalar

    # solution data
    need_rebuild::Bool
    need_rebuild_names_ctr::Bool
    need_rebuild_names_var::Bool
    solver#::MOI.ModelLike
    mode
    upper_to_sblm
    lower_to_sblm
    lower_dual_to_sblm
    sblm_to_solver
    lower_primal_dual_map

    solve_time::Float64
    build_time::Float64

    copy_names::Bool

    objdict::Dict{Symbol, Any}    # Same that JuMP.Model's field `objdict`

    function BilevelModel()

        model = new(
            JuMP.Model(),
            JuMP.Model(),

            # var
            0, Dict{Int, JuMP.AbstractVariable}(),
            Dict{Int, String}(),Dict{String, Int}(),
            Dict{Int, Level}(), Dict{Int, JuMP.AbstractVariable}(), Dict{Int, JuMP.AbstractVariable}(),
            Dict{Int, VariableInfo}(),
            # links
            Dict{JuMP.AbstractVariable, JuMP.AbstractVariable}(), Dict{JuMP.AbstractVariable, JuMP.AbstractVariable}(),
            Dict{JuMP.AbstractVariable, JuMP.ConstraintRef}(),
            Dict{JuMP.AbstractVariable, JuMP.AbstractVariable}(),
            #ctr
            0, Dict{Int, JuMP.AbstractConstraint}(),
            Dict{Int, String}(),Dict{String, Int}(),
            Dict{Int, Level}(), Dict{Int, JuMP.AbstractConstraint}(), Dict{Int, JuMP.AbstractConstraint}(),
            Dict{Int, ConstraintInfo}(),
            #obj
            MOI.FEASIBILITY_SENSE,
            zero(JuMP.GenericAffExpr{Float64, BilevelVariableRef}), # Model objective
            MOI.FEASIBILITY_SENSE,
            zero(JuMP.GenericAffExpr{Float64, BilevelVariableRef}), # Model objective

            true,
            true,
            true,
            nothing,
            NoMode{Float64},
            nothing,
            nothing,
            nothing,
            nothing,
            nothing,

            NaN,
            NaN,
            false,
            Dict{Symbol, Any}(),
            )

        return model
    end
end
function BilevelModel(optimizer_constructor;
    bridge_constraints::Bool=true, mode::BilevelSolverMode = SOS1Mode())
    bm = BilevelModel()
    set_mode(bm, mode)
    JuMP.set_optimizer(bm, optimizer_constructor; bridge_constraints=bridge_constraints)
    return bm
end
function set_mode(bm::BilevelModel, mode::BilevelSolverMode)
    bm.mode = mode
    return bm
end

abstract type InnerBilevelModel <: AbstractBilevelModel end
struct UpperModel <: InnerBilevelModel
    m::BilevelModel
end
Upper(m::BilevelModel) = UpperModel(m)
UpperToLower(m::BilevelModel) = UpperModel(m)
struct LowerModel <: InnerBilevelModel
    m::BilevelModel
end
Lower(m::BilevelModel) = LowerModel(m)
LowerToUpper(m::BilevelModel) = LowerModel(m)
bilevel_model(m::InnerBilevelModel) = m.m
mylevel_model(m::UpperModel) = bilevel_model(m).upper
mylevel_model(m::LowerModel) = bilevel_model(m).lower
level(m::LowerModel) = LOWER_ONLY
level(m::UpperModel) = UPPER_ONLY
mylevel_ctr_list(m::LowerModel) = bilevel_model(m).ctr_lower
mylevel_ctr_list(m::UpperModel) = bilevel_model(m).ctr_upper
mylevel_var_list(m::LowerModel) = bilevel_model(m).var_lower
mylevel_var_list(m::UpperModel) = bilevel_model(m).var_upper

# obj

mylevel_obj_sense(m::LowerModel) = bilevel_model(m).lower_objective_sense
mylevel_obj_function(m::LowerModel) = bilevel_model(m).lower_objective_function
mylevel_obj_sense(m::UpperModel) = bilevel_model(m).upper_objective_sense
mylevel_obj_function(m::UpperModel) = bilevel_model(m).upper_objective_function

set_mylevel_obj_sense(m::LowerModel, val) = bilevel_model(m).lower_objective_sense = val
set_mylevel_obj_function(m::LowerModel, val) = bilevel_model(m).lower_objective_function = val
set_mylevel_obj_sense(m::UpperModel, val) = bilevel_model(m).upper_objective_sense = val
set_mylevel_obj_function(m::UpperModel, val) = bilevel_model(m).upper_objective_function = val

function set_link!(m::UpperModel, upper::JuMP.AbstractVariableRef, lower::JuMP.AbstractVariableRef)
    bilevel_model(m).upper_to_lower_link[upper] = lower
    bilevel_model(m).link[upper] = lower
    nothing
end
function set_link!(m::LowerModel, upper::JuMP.AbstractVariableRef, lower::JuMP.AbstractVariableRef)
    bilevel_model(m).lower_to_upper_link[lower] = upper
    bilevel_model(m).link[upper] = lower
    nothing
end

# Models to deal with variables that are not exchanged between models
abstract type SingleBilevelModel <: AbstractBilevelModel end
struct UpperOnlyModel <: SingleBilevelModel
    m::BilevelModel
end
UpperOnly(m::BilevelModel) = UpperOnlyModel(m)
struct LowerOnlyModel <: SingleBilevelModel
    m::BilevelModel
end
LowerOnly(m::BilevelModel) = LowerOnlyModel(m)

bilevel_model(m::SingleBilevelModel) = m.m
mylevel_model(m::UpperOnlyModel) = bilevel_model(m).upper
mylevel_model(m::LowerOnlyModel) = bilevel_model(m).lower
level(::LowerOnlyModel) = LOWER_ONLY
level(::UpperOnlyModel) = UPPER_ONLY
mylevel_var_list(m::LowerOnlyModel) = bilevel_model(m).var_lower
mylevel_var_list(m::UpperOnlyModel) = bilevel_model(m).var_upper

in_upper(l::Level) = l == BOTH || l == UPPER_ONLY || l == DUAL_OF_LOWER
in_lower(l::Level) = l == BOTH || l == LOWER_ONLY

#### Model ####



# Variables
struct BilevelVariableRef <: JuMP.AbstractVariableRef
    model::BilevelModel # `model` owning the variable
    idx::Int       # Index in `model.variables`
    level::Level
end
function BilevelVariableRef(model, idx)
    return BilevelVariableRef(model, idx, model.var_level[idx])
end

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

# Constraints
# struct BilevelConstraintRef
#     model::BilevelModel # `model` owning the constraint
#     idx::Int       # Index in `model.constraints`
#     level::Level
# end
const BilevelConstraintRef = JuMP.ConstraintRef{BilevelModel, Int}#, Shape <: AbstractShape
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
    # @show c
    # @show func
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
function MOI.delete!(m::AbstractBilevelModel, cref::BilevelConstraintRef)
    error("can't delete")
    m.need_rebuild = true
    delete!(m.constraints, cref.index)
    delete!(m.connames, cref.index)
end
MOI.is_valid(m::BilevelModel, cref::BilevelConstraintRef) = cref.index in keys(m.constraints)
MOI.is_valid(m::InnerBilevelModel, cref::BilevelConstraintRef) =
    MOI.is_valid(bilevel_model(m), cref) && level(cref) == level(m)
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

function set_dual_start(cref::BilevelConstraintRef, value::T) where T<:Number
    cref.model.ctr_info[cref.index].start = value
end
function set_dual_start(cref::BilevelConstraintRef, value::T) where T<:Vector{S} where S
    array = cref.model.ctr_info[cref.index].start
    @assert length(array) == length(value)
    copyto!(array, value)
end
function get_dual_start(cref::BilevelConstraintRef)
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
        set_dual_start(dual_of.ci, info.start)
        # info.has_start = false
        # info.start = NaN
    end
    info.binary    && _error("Dual variable cannot be binary")
    info.integer   && _error("Dual variable cannot be integer")

    info = JuMP.VariableInfo(false, NaN, false, NaN,
                             false, NaN, false, NaN,
                             false, false)

    if level(dual_of.ci) != LOWER_ONLY
        error("Variables can only be tied to LOWER level constraints, got $(dual_of.ci.level) level")
    end

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
    return VariableInfo{Float64}()
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
    return BilevelConstraintRef(model, idx, DUAL_OF_LOWER)
end

# Objective
function JuMP.set_objective(m::InnerBilevelModel, sense::MOI.OptimizationSense,
                            f::JuMP.AbstractJuMPScalar)
    set_mylevel_obj_sense(m, sense)
    set_mylevel_obj_function(m, f)
    level_f = replace_variables(f, bilevel_model(m), mylevel_model(m), mylevel_var_list(m), level(m))
    JuMP.set_objective(mylevel_model(m), sense, level_f)
end
JuMP.objective_sense(m::InnerBilevelModel) = mylevel_obj_sense(m)
JuMP.objective_function_type(m::InnerBilevelModel) = typeof(mylevel_obj_function(m))
JuMP.objective_function(m::InnerBilevelModel) = mylevel_obj_function(m)
function JuMP.objective_function(m::InnerBilevelModel, FT::Type)
    mylevel_obj_function(m) isa FT || error("The objective function is not of type $FT")
    mylevel_obj_function(m)
end

function JuMP.relative_gap(bm::BilevelModel)::Float64
    _check_solver(bm)
    return MOI.get(bm.solver, MOI.RelativeGap())
end
function JuMP.relative_gap(bm::UpperModel)::Float64
    return JuMP.relative_gap(bm.m)
end
function JuMP.dual_objective_value(bm::BilevelModel; result::Int = 1)::Float64
    _check_solver(bm)
    return MOI.get(bm.solver, MOI.DualObjectiveValue(result))
end
function JuMP.objective_bound(bm::BilevelModel)::Float64
    _check_solver(bm)
    return MOI.get(bm.solver, MOI.ObjectiveBound())
end
function JuMP.objective_bound(bm::UpperModel)::Float64
    return JuMP.objective_bound(bm.m)
end

JuMP.num_variables(m::AbstractBilevelModel) = JuMP.num_variables(bilevel_model(m))
_plural(n) = (isone(n) ? "" : "s")
function JuMP.show_constraints_summary(io::IO, model::BilevelModel)
    JuMP.show_constraints_summary(io, Upper(model))
    JuMP.show_constraints_summary(io, Lower(model), true)
end
function JuMP.show_constraints_summary(io::IO, model::UpperModel)
    n = JuMP.num_constraints(model)
    println(io, "Upper Constraint", _plural(n), ": ", n)
end
function JuMP.show_constraints_summary(io::IO, model::LowerModel, line_break = false)
    n = JuMP.num_constraints(model)
    print(io, "Lower Constraint", _plural(n), ": ", n, ifelse(line_break, "\n",""))
end
JuMP.show_backend_summary(io::IO, model::InnerBilevelModel) = JuMP.show_backend_summary(io, model.m)
function JuMP.show_backend_summary(io::IO, model::BilevelModel)
    println(io, "Bilevel Model")
    println(io, "Solution method: ", model.mode)
    if model.solver === nothing
        print(io, "No solver attached")
    else
        name = JuMP.solver_name(model::BilevelModel)
        print(io, "Solver name: ", name)
    end
end
function JuMP.solver_name(model::BilevelModel)
    name = try
        MOI.get(model.solver, MOI.SolverName())::String
    catch ex
        if isa(ex, ArgumentError)
            return "SolverName() attribute not implemented by the optimizer."
        else
            rethrow(ex)
        end
    end
end

JuMP.object_dictionary(m::BilevelModel) = m.objdict
JuMP.object_dictionary(m::AbstractBilevelModel) = JuMP.object_dictionary(bilevel_model(m))

function JuMP.show_objective_function_summary(io::IO, model::UpperModel)
    println(io, "Upper objective function type: \n",
            JuMP.objective_function_type(model))
end
function JuMP.show_objective_function_summary(io::IO, model::LowerModel)
    println(io, "Lower objective function type: \n",
            JuMP.objective_function_type(model))
end
function JuMP.show_objective_function_summary(io::IO, model::BilevelModel)
    JuMP.show_objective_function_summary(io, Upper(model))
    JuMP.show_objective_function_summary(io, Lower(model))
end

bilevel_obj_error() = error("There is no objective for BilevelModel use Upper(.) and Lower(.)")

function JuMP.set_objective(m::BilevelModel, sense::MOI.OptimizationSense,
    f::JuMP.AbstractJuMPScalar)
    bilevel_obj_error()
end
JuMP.objective_sense(m::BilevelModel) = JuMP.objective_sense(m.upper)#bilevel_obj_error()
JuMP.objective_function_type(model::BilevelModel) = bilevel_obj_error()
JuMP.objective_function(model::BilevelModel) = bilevel_obj_error()
function JuMP.objective_function(model::BilevelModel, FT::Type)
    bilevel_obj_error()
end

# Names
JuMP.name(vref::BilevelVariableRef) = vref.model.varnames[vref.idx]
function JuMP.set_name(vref::BilevelVariableRef, name::String)
    vref.model.varnames[vref.idx] = name
end
JuMP.name(cref::BilevelConstraintRef) = cref.model.connames[cref.index]
function JuMP.set_name(cref::BilevelConstraintRef, name::String)
    cref.model.connames[cref.index] = name
end
function JuMP.variable_by_name(model::BilevelModel, name::String)
    if model.need_rebuild_names_var
        model.varnames_rev = Dict(v => k for (k,v) in model.varnames)
    end
    idx = model.varnames_rev[name]
    return BilevelVariableRef(model, idx)
end
function JuMP.constraint_by_name(model::BilevelModel, name::String)
    if model.need_rebuild_names_ctr
        model.connames_rev = Dict(v => k for (k,v) in model.connames)
    end
    idx = model.connames_rev[name]
    return BilevelConstraintRef(model, idx)
end

# replace variables
function replace_variables(var::BilevelVariableRef,
    model::BilevelModel,
    inner::JuMP.AbstractModel,
    variable_map::Dict{Int, V},
    level::Level) where {V<:JuMP.AbstractVariableRef}
    if var.model === model && in_level(var, level)
        return variable_map[var.idx]
    elseif var.model === model
        error("Variable $(var) belonging Only to $(var.level) level, was added in the $(level) level.")
    else
        error("A BilevelModel cannot have expression using variables of a BilevelModel different from itself")
    end
end
function replace_variables(aff::JuMP.GenericAffExpr{C, BilevelVariableRef},
    model::BilevelModel,
    inner::JuMP.AbstractModel,
    variable_map::Dict{Int, V},
    level::Level) where {C,V<:JuMP.AbstractVariableRef}
    result = JuMP.GenericAffExpr{C, JuMP.VariableRef}(0.0)#zero(aff)
    result.constant = aff.constant
    for (coef, var) in JuMP.linear_terms(aff)
        JuMP.add_to_expression!(result,
        coef,
        replace_variables(var, model, model, variable_map, level))
    end
    return result
end
function replace_variables(quad::JuMP.GenericQuadExpr{C, BilevelVariableRef},
    model::BilevelModel,
    inner::JuMP.AbstractModel,
    variable_map::Dict{Int, V},
    level::Level) where {C,V<:JuMP.AbstractVariableRef}
    aff = replace_variables(quad.aff, model, model, variable_map, level)
    quadv = JuMP.GenericQuadExpr{C, JuMP.VariableRef}(aff)
    for (coef, var1, var2) in JuMP.quad_terms(quad)
        JuMP.add_to_expression!(quadv,
        coef,
        replace_variables(var1, model, model, variable_map, level),
        replace_variables(var2, model, model, variable_map, level))
    end
    return quadv
end
replace_variables(funcs::Vector, args...) = map(f -> replace_variables(f, args...), funcs)

function print_lp(m, name)
    dest = MOI.FileFormats.Model(format = MOI.FileFormats.FORMAT_MOF)
    MOI.copy_to(dest, m)
    MOI.write_to_file(dest, name)
end

JuMP.optimize!(::T) where {T<:AbstractBilevelModel} =
    error("Can't solve a model of type: $T ")
function JuMP.optimize!(model::BilevelModel;
    lower_prob = "", upper_prob = "", bilevel_prob = "", solver_prob = "")

    if model.mode === nothing
        error("No solution mode selected, use `set_mode(model, mode)` or initialize with `BilevelModel(optimizer_constructor, mode = some_mode)`")
    else
        mode = model.mode
    end

    _check_solver(model)

    solver = model.solver #optimizer#MOI.Bridges.full_bridge_optimizer(optimizer, Float64)
    if true#!MOI.is_empty(solver)
        MOI.empty!(solver)
    end

    upper = JuMP.backend(model.upper)
    lower = JuMP.backend(model.lower)

    if length(lower_prob) > 0
        print_lp(lower, lower_prob)
    end
    if length(upper_prob) > 0
        print_lp(upper, upper_prob)
    end

    t0 = time()

    moi_upper = JuMP.index.(
        collect(values(model.upper_to_lower_link)))
    moi_link = JuMP.index(model.link)
    moi_link2 = index2(model.upper_var_to_lower_ctr_link)

    # build bound for FortunyAmatMcCarlMode
    build_bounds!(model, mode)

    single_blm, upper_to_sblm, lower_to_sblm, lower_primal_dual_map, lower_dual_to_sblm =
    build_bilevel(upper, lower, moi_link, moi_upper, mode, moi_link2)

    # pass lower level dual variables info (start, upper, lower)
    for (idx, info) in model.ctr_info
        if haskey(model.ctr_lower, idx)
            ctr = model.ctr_lower[idx]
            pre_duals = lower_primal_dual_map.primal_con_dual_var[JuMP.index(ctr)] # vector
            duals = map(x->lower_dual_to_sblm[x], pre_duals)
            pass_dual_info(single_blm, duals, info)
        end
    end
    # pass lower & upper level primal variables info (upper, lower)
    for (idx, info) in model.var_info
        if haskey(model.var_lower, idx)
            var = lower_to_sblm[JuMP.index(model.var_lower[idx])]
        elseif haskey(model.var_upper, idx)
            var = upper_to_sblm[JuMP.index(model.var_upper[idx])]
        else
            continue
        end
        pass_primal_info(single_blm, var, info)
    end
    if length(bilevel_prob) > 0
        print_lp(single_blm, bilevel_prob)
    end
    # print_lp(single_blm, "bilevel_orig.mof")

    sblm_to_solver = MOI.copy_to(solver, single_blm, copy_names = model.copy_names)
    # print_lp(solver, "bilevel_bridge.mof")
    # print_lp(solver.model, "bilevel_cache.mof")

    if length(solver_prob) > 0
        print_lp(solver, solver_prob)
    end

    # model.mode = mode
    # model.solver = solver#::MOI.ModelLike
    model.upper_to_sblm = upper_to_sblm
    model.lower_to_sblm = lower_to_sblm
    model.lower_dual_to_sblm = lower_dual_to_sblm
    model.lower_primal_dual_map = lower_primal_dual_map
    model.sblm_to_solver = sblm_to_solver

    t1 = time()
    model.build_time = t1 - t0

    MOI.optimize!(solver)

    model.solve_time = time() - t1

    return nothing
end
function pass_dual_info(single_blm, dual, info::ConstraintInfo{Float64})
    if !isnan(info.start)
        MOI.set(single_blm, MOI.VariablePrimalStart(), dual[], info.start)
    end
    if !isnan(info.upper) &&
        !MOI.is_valid(single_blm, CI{SVF,LT{Float64}}(dual[].value))
        MOI.add_constraint(single_blm,
            SVF(dual[]), LT{Float64}(info.upper))
    end
    if !isnan(info.lower) &&
        !MOI.is_valid(single_blm, CI{SVF,GT{Float64}}(dual[].value))
        MOI.add_constraint(single_blm,
            SVF(dual[]), GT{Float64}(info.lower))
    end
    return
end
function pass_dual_info(single_blm, dual, info::ConstraintInfo{Vector{Float64}})
    for i in eachindex(dual)
        if !isnan(info.start[i])
            MOI.set(single_blm, MOI.VariablePrimalStart(), dual[i], info.start[i])
        end
        if !isnan(info.upper[i]) &&
            !MOI.is_valid(single_blm, CI{SVF,LT{Float64}}(dual[i].value))
            MOI.add_constraint(single_blm,
                SVF(dual[i]), LT{Float64}(info.upper[i]))
        end
        if !isnan(info.lower[i]) &&
            !MOI.is_valid(single_blm, CI{SVF,GT{Float64}}(dual[i].value))
            MOI.add_constraint(single_blm,
                SVF(dual[i]), MOI.GreaterThan{Float64}(info.lower[i]))
        end
    end
    return
end

function pass_primal_info(single_blm, primal, info::VariableInfo{Float64})
    if !isnan(info.upper) &&
        !MOI.is_valid(single_blm, CI{SVF,LT{Float64}}(primal.value))
        MOI.add_constraint(single_blm,
            SVF(primal), LT{Float64}(info.upper))
    end
    if !isnan(info.lower) &&
        !MOI.is_valid(single_blm, CI{SVF,GT{Float64}}(primal.value))
        MOI.add_constraint(single_blm,
            SVF(primal), GT{Float64}(info.lower))
    end
    return
end

function JuMP.index(d::Dict)
    ret = Dict{VI,VI}()
    # sizehint!(ret, length(d))
    for (k,v) in d
        ret[JuMP.index(k)] = JuMP.index(v)
    end
    return ret
end
function index2(d::Dict)
    ret = Dict{VI,CI}()
    # sizehint!(ret, length(d))
    for (k,v) in d
        ret[JuMP.index(k)] = JuMP.index(v)
    end
    return ret
end

function JuMP.value(v::BilevelVariableRef; result::Int = 1)::Float64
    m = owner_model(v)
    solver = m.solver
    ref = solver_ref(v)
    return MOI.get(solver, MOI.VariablePrimal(result), ref)
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

function JuMP.primal_status(model::BilevelModel)
    _check_solver(model)
    return MOI.get(model.solver, MOI.PrimalStatus())
end
function JuMP.primal_status(model::InnerBilevelModel)
    return JuMP.primal_status(model.m)
end

JuMP.dual_status(::BilevelModel) = error(
    "Dual status cant be queried for BilevelModel, but you can query for Upper and Lower models.")
function JuMP.dual_status(model::UpperModel)
    _check_solver(model.m)
    return MOI.get(model.m.solver, MOI.DualStatus())
end
function JuMP.dual_status(model::LowerModel)
    _check_solver(model.m)
    return MOI.get(model.m.solver, MOI.PrimalStatus())
end

function JuMP.termination_status(model::BilevelModel)
    _check_solver(model)
    return MOI.get(model.solver, MOI.TerminationStatus())
end
function JuMP.raw_status(model::BilevelModel)
    _check_solver(model)
    return MOI.get(model.solver, MOI.RawStatusString())
end

function JuMP.objective_value(model::BilevelModel)
    _check_solver(model)
    return MOI.get(model.solver, MOI.ObjectiveValue())
end
function JuMP.objective_value(model::UpperModel)
    return JuMP.objective_value(model.m)
end
function JuMP.objective_value(model::LowerModel)
    return lower_objective_value(model.m)
end

# function inner_ref_to_value(lm::LowerModel, var)
#     m = lm.m
#     ref = m.sblm_to_solver[m.lower_to_sblm[JuMP.index(var)]]
#     return MOI.get(m.solver, MOI.VariablePrimal(), ref)
# end
function lower_objective_value(model::BilevelModel; result::Int = 1)
    f = JuMP.objective_function(Lower(model))
    # Evaluate the lower objective expression
    return JuMP.value(f, v -> JuMP.value(v, result = result))
    # return JuMP.value(v -> inner_ref_to_value(Lower(model), v), f)
end

function build_bounds!(::BilevelModel, ::BilevelSolverMode{T}) where T
    return nothing
end

function bigM(is_dual, mode::FortunyAmatMcCarlMode)
    return ifelse(is_dual, mode.dual_big_M, mode.primal_big_M)
end
function inf_if_nan(::typeof(+), val)
    ifelse(isnan(val), Inf, val)
end
function inf_if_nan(::typeof(-), val)
    ifelse(isnan(val), -Inf, val)
end
function build_bounds!(model::BilevelModel, mode::FortunyAmatMcCarlMode{T}) where T
    # compute variable bounds for FA mode
    fa_vi_up = mode.upper
    fa_vi_lo = mode.lower
    fa_vi_ld = mode.ldual
    empty!(fa_vi_up)
    empty!(fa_vi_lo)
    empty!(fa_vi_ld)
    for (idx, _info) in model.var_info
        if haskey(model.var_lower, idx)
            var = JuMP.index(model.var_lower[idx])
            is_lower = true
        elseif haskey(model.var_upper, idx)
            var = JuMP.index(model.var_upper[idx])
            is_lower = false
        else
            continue
        end
        vref = BilevelVariableRef(model, idx)

        is_dual = vref.level == DUAL_OF_LOWER

        info = deepcopy(_info)

        lb = JuMP.has_lower_bound(vref) ? JuMP.lower_bound(vref) : -Inf
        ub = JuMP.has_upper_bound(vref) ? JuMP.upper_bound(vref) : +Inf
        info.upper = min(ub, inf_if_nan(+, info.upper))
        if info.upper == +Inf
            info.upper = +bigM(is_dual, mode)
        end
        info.lower = max(lb, inf_if_nan(-, info.lower))
        if info.lower == -Inf
            info.lower = -bigM(is_dual, mode)
        end
        if info.upper == +Inf && mode.safe
            error("Problem with UB of variable $vref")
        end
        if info.lower == -Inf && mode.safe
            error("Problem with LB of variable $vref")
        end
        if is_lower
            fa_vi_lo[var] = info
        else
            fa_vi_up[var] = info
        end
        @assert info.lower <= info.upper

    end
    for (idx, _info) in model.ctr_info
        if haskey(model.ctr_lower, idx)
            ctr = JuMP.index(model.ctr_lower[idx])
            info = deepcopy(_info)
            cref = BilevelConstraintRef(model, idx)
            # scalar
            ub = dual_upper_bound(ctr)
            lb = dual_lower_bound(ctr)
            info.upper = min(ub, inf_if_nan(+, info.upper))
            if info.upper == +Inf
                info.upper = +bigM(true, mode)
            end
            info.lower = max(lb, inf_if_nan(-, info.lower))
            if info.lower == -Inf
                info.lower = -bigM(true, mode)
            end
            if info.upper == +Inf && mode.safe
                error("Problem with UB of dual of constraint $cref")
            end
            if info.lower == -Inf && mode.safe
                error("Problem with LB of dual of constraint $cref")
            end
            @assert info.lower <= info.upper
            # TODO vector
            fa_vi_ld[ctr] = info
        end
    end
    return nothing
end

dual_lower_bound(::CI{F,LT{T}}) where {F,T} = -Inf
dual_upper_bound(::CI{F,LT{T}}) where {F,T} =  0.0

dual_lower_bound(::CI{F,GT{T}}) where {F,T} =  0.0
dual_upper_bound(::CI{F,GT{T}}) where {F,T} = +Inf

dual_lower_bound(::CI{F,ET{T}}) where {F,T} =  0.0
dual_upper_bound(::CI{F,ET{T}}) where {F,T} =  0.0

dual_lower_bound(::CI{F,S}) where {F,S} = -Inf
dual_upper_bound(::CI{F,S}) where {F,S} = +Inf

function JuMP.solve_time(bm::BilevelModel)
    return bm.solve_time
end
function build_time(bm::BilevelModel)
    return bm.build_time
end

function JuMP.set_optimizer_attribute(bm::BilevelModel, name::String, value)
    _check_solver(bm)
    return JuMP.set_optimizer_attribute(bm, MOI.RawParameter(name), value)
end
function JuMP.set_optimizer_attribute(
    bm::BilevelModel, attr::MOI.AbstractOptimizerAttribute, value
)
    _check_solver(bm)
    return MOI.set(bm.solver, attr, value)
end
function JuMP.set_optimizer_attributes(bm::BilevelModel, pairs::Pair...)
    for (name, value) in pairs
        JuMP.set_optimizer_attribute(bm.solver, name, value)
    end
end

function JuMP.get_optimizer_attribute(bm::BilevelModel, name::String)
    _check_solver(bm)
    return JuMP.get_optimizer_attribute(bm.solver, MOI.RawParameter(name))
end
function JuMP.get_optimizer_attribute(
    bm::BilevelModel, attr::MOI.AbstractOptimizerAttribute
)
    _check_solver(bm)
    return MOI.get(bm.solver, attr)
end

function JuMP.set_silent(bm::BilevelModel)
    _check_solver(bm)
    return MOI.set(bm.solver, MOI.Silent(), true)
end

function JuMP.unset_silent(bm::BilevelModel)
    _check_solver(bm)
    return MOI.set(bm.solver, MOI.Silent(), false)
end

function JuMP.set_time_limit_sec(bm::BilevelModel, limit)
    _check_solver(bm)
    return MOI.set(bm.solver, MOI.TimeLimitSec(), limit)
end

function JuMP.unset_time_limit_sec(bm::BilevelModel)
    _check_solver(bm)
    return MOI.set(bm.solver, MOI.TimeLimitSec(), nothing)
end

function JuMP.time_limit_sec(bm::BilevelModel)
    _check_solver(bm)
    return MOI.get(bm.solver, MOI.TimeLimitSec())
end

function JuMP.simplex_iterations(bm::BilevelModel)
    _check_solver(bm)
    return MOI.get(bm.solver, MOI.SimplexIterations())
end

function JuMP.barrier_iterations(bm::BilevelModel)
    _check_solver(bm)
    return MOI.get(bm.solver, MOI.BarrierIterations())
end

function JuMP.node_count(bm::BilevelModel)
    _check_solver(bm)
    return MOI.get(bm.solver, MOI.NodeCount())
end

function JuMP.normalized_rhs(cref::BilevelConstraintRef)
    return JuMP.normalized_rhs(raw_ref(cref))
end

function JuMP.result_count(bm::BilevelModel)::Int
    _check_solver(bm)
    return MOI.get(bm.solver, MOI.ResultCount())
end


function _check_solver(bm::BilevelModel)
    if bm.solver === nothing
        error("No solver attached, use `set_optimizer(model, optimizer_constructor)` or initialize with `BilevelModel(optimizer_constructor)`")
    end
end

function JuMP.set_optimizer(bm::BilevelModel, optimizer_constructor;
    bridge_constraints::Bool=true)
    # error_if_direct_mode(model, :set_optimizer)
    if bridge_constraints
        # We set `with_names=false` because the names are handled by the first
        # caching optimizer. If `default_copy_to` without names is supported,
        # no need for a second cache.
        optimizer = MOI.instantiate(optimizer_constructor, with_bridge_type=Float64, with_names=false)
        # for bridge_type in model.bridge_types
        #     _moi_add_bridge(optimizer, bridge_type)
        # end
    else
        optimizer = MOI.instantiate(optimizer_constructor)
    end

    bm.solver = optimizer
    if !MOI.is_empty(bm.solver)
        error("Calling the `optimizer_constructor` must return an empty optimizer")
    end
    return bm
end
