
# The following is largely inspired from JuMP/test/JuMPExtension.jl

@enum Level BOTH LOWER_ONLY UPPER_ONLY DUAL_OF_LOWER

abstract type AbstractBilevelModel <: JuMP.AbstractModel end

Base.broadcastable(model::AbstractBilevelModel) = Ref(model)

mutable struct BilevelModel <: AbstractBilevelModel
    # Structured data
    # JuMP models that hold data for each of the two levels
    # constraints and objectives only appear in the named level
    # (Upper/Lower)Only variable also appear only on the named level
    # other variables (linking from both sides) appear on both
    # linking variable must be differentiated by other methods
    upper::JuMP.AbstractModel
    lower::JuMP.AbstractModel

    # Model data
    # Integer index of the last variable added (for indexing BilevelVariableRef's)
    last_variable_index::Int
    # Maps indexes to JuMP.ScalarVariable
    # structure that holds basic data for variables
    # such as start values, bounds, integrality
    # TODO remove and rely on MOI
    variables::Dict{Int, JuMP.AbstractVariable}     # Map varidx -> variable

    # storage for variable names
    # TODO remove and rely on MOI
    varnames::Dict{Int, String}                     # Map varidx -> name
    varnames_rev::Dict{String, Int}                 # Map varidx -> name

    # holds the variable level: BOTH LOWER_ONLY UPPER_ONLY DUAL_OF_LOWER
    var_level::Dict{Int, Level}

    # maps the BilevelVariableRef index
    # to JuMP variables of the correct level
    # variable that appear in both levels are inboth dicts
    var_upper::Dict{Int, JuMP.AbstractVariableRef}
    var_lower::Dict{Int, JuMP.AbstractVariableRef}

    # additional info for variables such as bound hints
    # TODO merge with `var_level` and `variables`
    var_info::Dict{Int, VariableInfo}

    # maps JuMP.VariableRef to BilevelVariableRef
    # built upon necessity for getting contraints and functions
    var_upper_rev::Union{Nothing, Dict{JuMP.AbstractVariableRef, JuMP.AbstractVariableRef}}
    var_lower_rev::Union{Nothing, Dict{JuMP.AbstractVariableRef, JuMP.AbstractVariableRef}}

    # JuMP.VariableRef of variables from named level that are NOT linking
    upper_only::Set{JuMP.AbstractVariableRef}
    lower_only::Set{JuMP.AbstractVariableRef}

    # upper level decisions that are "parameters" of the second level
    # keys are *decision* variables from the upper level
    # values are *parameter* variables from the lower level (linked to the keys)
    upper_to_lower_link::Dict{JuMP.AbstractVariableRef, JuMP.AbstractVariableRef}
    # lower level decisions that are input to upper
    # keys are *decision* variables from the lower level
    # values are *parameter* variables from the upper level (linked to the keys)
    lower_to_upper_link::Dict{JuMP.AbstractVariableRef, JuMP.AbstractVariableRef}
    # lower level decisions that are input to upper
    # keys are upper level variables (representing lower level dual variables)
    # values are lower level constraints
    upper_var_to_lower_ctr_link::Dict{JuMP.AbstractVariableRef, JuMP.ConstraintRef}
    # joint link
    # for all variables that appear in both models
    # keys are upper indices and values are lower indices
    # same as: merge(upper_to_lower_link, reverse(lower_to_upper_link))
    link::Dict{JuMP.AbstractVariableRef, JuMP.AbstractVariableRef}

    # Integer index of the last constraint added (for indexing BilevelConstraintRef's)
    nextconidx::Int

    # holds JuMP.(Scalar/Vector)Constraint{F,S}
    # only used in: JuMP.constraint_object
    constraints::Dict{Int, JuMP.AbstractConstraint} # Map conidx -> variable

    # storage for constraint names
    # TODO remove and rely on MOI
    connames::Dict{Int, String}                     # Map varidx -> name
    connames_rev::Dict{String, Int}                 # Map varidx -> name

    # holds the variable level: LOWER_ONLY UPPER_ONLY
    ctr_level::Dict{Int, Level}

    # maps the BilevelConstraintRef index
    # to JuMP ConstraintRef of the correct level
    ctr_upper::Dict{Int, JuMP.ConstraintRef}
    ctr_lower::Dict{Int, JuMP.ConstraintRef}

    # additional info for constraints such as bound hints and start values
    # TODO merge with `ctr_level` and `constraints`
    ctr_info::Dict{Int, ConstraintInfo}

    # maps JuMP.ConstraintRef to BilevelConstraintRef
    # built upon necessity for getting contraints and functions
    ctr_upper_rev::Union{Nothing, Dict{JuMP.ConstraintRef, JuMP.ConstraintRef}} # bilevel ref no defined
    ctr_lower_rev::Union{Nothing, Dict{JuMP.ConstraintRef, JuMP.ConstraintRef}} # bilevel ref no defined

    # solution data
    need_rebuild::Bool
    need_rebuild_names_ctr::Bool
    need_rebuild_names_var::Bool

    #
    solver#::MOI.ModelLike
    mode

    # maps for MPEC based solution methods
    # from upper level JuMP.index(JuMP.VariableRef) = (MOI.VI)
    # to mpec indices
    upper_to_sblm
    # from lower MOI indices
    # to mpec indices
    lower_to_sblm
    # from lower dual MOI indices
    # to mpec indices
    lower_dual_to_sblm
    # from mped indices to solver indices
    sblm_to_solver
    # lower primal to dual map
    # to obtain dual variables from primal constraints
    lower_primal_dual_map

    # results from opt process
    solve_time::Float64
    build_time::Float64

    # BilevelModel model attributes
    copy_names::Bool
    copy_names_to_solver::Bool
    pass_start::Bool

    # for completing the JuMP.Model API
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
            nothing,
            nothing,
            # links
            Set{JuMP.AbstractVariableRef}(),
            Set{JuMP.AbstractVariableRef}(),
            Dict{JuMP.AbstractVariable, JuMP.AbstractVariable}(),
            Dict{JuMP.AbstractVariable, JuMP.AbstractVariable}(),
            Dict{JuMP.AbstractVariable, JuMP.ConstraintRef}(),
            Dict{JuMP.AbstractVariable, JuMP.AbstractVariable}(),
            #ctr
            0, Dict{Int, JuMP.AbstractConstraint}(),
            Dict{Int, String}(),Dict{String, Int}(),
            Dict{Int, Level}(), Dict{Int, JuMP.AbstractConstraint}(), Dict{Int, JuMP.AbstractConstraint}(),
            Dict{Int, ConstraintInfo}(),
            nothing,
            nothing,

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
            false,
            true,
            Dict{Symbol, Any}(),
            )

        return model
    end
end
function BilevelModel(optimizer_constructor;
    mode::AbstractBilevelSolverMode = SOS1Mode(),
    bridge_constraints::Bool=true,
    with_names=true)
    bm = BilevelModel()
    bm.copy_names=with_names
    set_mode(bm, mode)
    JuMP.set_optimizer(bm, optimizer_constructor;
        bridge_constraints=bridge_constraints,
        with_names=with_names)
    return bm
end
function set_mode(bm::BilevelModel, mode::AbstractBilevelSolverMode)
    bm.mode = deepcopy(mode)
    reset!(bm.mode)
    return bm
end

abstract type InnerBilevelModel <: AbstractBilevelModel end
struct UpperModel <: InnerBilevelModel
    m::BilevelModel
end
Upper(m::BilevelModel) = UpperModel(m)
struct LowerModel <: InnerBilevelModel
    m::BilevelModel
end
Lower(m::BilevelModel) = LowerModel(m)
bilevel_model(m::InnerBilevelModel) = m.m
mylevel_model(m::UpperModel) = bilevel_model(m).upper
mylevel_model(m::LowerModel) = bilevel_model(m).lower
level(::LowerModel) = LOWER_ONLY
level(::UpperModel) = UPPER_ONLY
mylevel_ctr_list(m::LowerModel) = bilevel_model(m).ctr_lower
mylevel_ctr_list(m::UpperModel) = bilevel_model(m).ctr_upper
mylevel_var_list(m::LowerModel) = bilevel_model(m).var_lower
mylevel_var_list(m::UpperModel) = bilevel_model(m).var_upper

# obj

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

function push_single_level_variable!(m::LowerOnlyModel, vref::JuMP.AbstractVariableRef)
    push!(bilevel_model(m).lower_only, vref)
end
function push_single_level_variable!(m::UpperOnlyModel, vref::JuMP.AbstractVariableRef)
    push!(bilevel_model(m).upper_only, vref)
end
#### Model ####

# Variables
struct BilevelVariableRef <: JuMP.AbstractVariableRef
    model::BilevelModel # `model` owning the variable
    idx::Int       # Index in `model.variables`
    level::Level
end
function BilevelVariableRef(model::BilevelModel, idx)
    return BilevelVariableRef(model, idx, model.var_level[idx])
end

# Constraints
const BilevelConstraintRef = JuMP.ConstraintRef{BilevelModel, Int}#, Shape <: AbstractShape

# Objective

# Etc

JuMP.object_dictionary(m::BilevelModel) = m.objdict
JuMP.object_dictionary(m::AbstractBilevelModel) = JuMP.object_dictionary(bilevel_model(m))

function convert_indices(d::Dict)
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

function JuMP.primal_status(model::BilevelModel)
    _check_solver(model)
    return MOI.get(model.solver, MOI.PrimalStatus())
end
function JuMP.primal_status(model::InnerBilevelModel)
    return JuMP.primal_status(model.m)
end

# Statuses

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

# Replace variables

replace_var_type(::Type{BilevelModel}) = JuMP.VariableRef
replace_var_type(::Type{M}) where {M<:JuMP.AbstractModel} = BilevelVariableRef
function build_reverse_var_map!(um::UpperModel)
    m = bilevel_model(um)
    m.var_upper_rev = Dict{JuMP.AbstractVariableRef, BilevelVariableRef}()
    for (idx, ref) in m.var_upper
        m.var_upper_rev[ref] = BilevelVariableRef(m, idx)
    end
end
function build_reverse_var_map!(lm::LowerModel)
    m = bilevel_model(lm)
    m.var_lower_rev = Dict{JuMP.AbstractVariableRef, BilevelVariableRef}()
    for (idx, ref) in m.var_lower
        m.var_lower_rev[ref] = BilevelVariableRef(m, idx)
    end
    return nothing
end
get_reverse_var_map(m::UpperModel) = m.m.var_upper_rev
get_reverse_var_map(m::LowerModel) = m.m.var_lower_rev
function reverse_replace_variable(f, m::InnerBilevelModel)
    build_reverse_var_map!(m)
    return replace_variables(f, mylevel_model(m), get_reverse_var_map(m), level(m))
end
function replace_variables(var::VV, # JuMP.VariableRef
    model::M,
    variable_map::Dict{I, V},
    level::Level) where {I,V<:JuMP.AbstractVariableRef, M, VV<:JuMP.AbstractVariableRef}
    return variable_map[var]
end
function replace_variables(var::BilevelVariableRef,
    model::M,
    variable_map::Dict{I, V},
    level::Level) where {I,V<:JuMP.AbstractVariableRef, M<:BilevelModel}
    if var.model === model && in_level(var, level)
        return variable_map[var.idx]
    elseif var.model === model
        error("Variable $(var) belonging Only to $(var.level) level, was added in the $(level) level.")
    else
        error("A BilevelModel cannot have expression using variables of a BilevelModel different from itself")
    end
end
function replace_variables(aff::JuMP.GenericAffExpr{C, VV},
    model::M,
    variable_map::Dict{I, V},
    level::Level) where {I,C,V<:JuMP.AbstractVariableRef, M, VV}
    result = JuMP.GenericAffExpr{C, replace_var_type(M)}(0.0)#zero(aff)
    result.constant = aff.constant
    for (coef, var) in JuMP.linear_terms(aff)
        JuMP.add_to_expression!(result,
        coef,
        replace_variables(var, model, variable_map, level))
    end
    return result
end
function replace_variables(quad::JuMP.GenericQuadExpr{C, VV},
    model::M,
    variable_map::Dict{I, V},
    level::Level) where {I,C,V<:JuMP.AbstractVariableRef, M, VV}
    aff = replace_variables(quad.aff, model, variable_map, level)
    quadv = JuMP.GenericQuadExpr{C, replace_var_type(M)}(aff)
    for (coef, var1, var2) in JuMP.quad_terms(quad)
        JuMP.add_to_expression!(quadv,
        coef,
        replace_variables(var1, model, variable_map, level),
        replace_variables(var2, model, variable_map, level))
    end
    return quadv
end
replace_variables(funcs::Vector, args...) = map(f -> replace_variables(f, args...), funcs)

function print_lp(m, name, file_format = MOI.FileFormats.FORMAT_AUTOMATIC)
    dest = MOI.FileFormats.Model(format = file_format, filename = name)
    MOI.copy_to(dest, m)
    MOI.write_to_file(dest, name)
end

# Optimize

JuMP.optimize!(::T) where {T<:AbstractBilevelModel} =
    error("Can't solve a model of type: $T ")
function JuMP.optimize!(model::BilevelModel;
    lower_prob = "", upper_prob = "", bilevel_prob = "", solver_prob = "", file_format = MOI.FileFormats.FORMAT_AUTOMATIC)

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

    if _has_nlp_data(model.upper)
        # this first NLPBlock passing is fake,
        # this is just necessary to force the variables
        # order to remain the same
        _load_nlp_data(model.upper)
    end

    upper = JuMP.backend(model.upper)
    lower = JuMP.backend(model.lower)

    if length(lower_prob) > 0
        print_lp(lower, lower_prob, file_format)
    end
    if length(upper_prob) > 0
        print_lp(upper, upper_prob, file_format)
    end

    t0 = time()

    moi_upper = JuMP.index.(
        collect(values(model.upper_to_lower_link)))
    moi_link = convert_indices(model.link)
    moi_link2 = index2(model.upper_var_to_lower_ctr_link)

    # build bound for FortunyAmatMcCarlMode
    build_bounds!(model, mode)

    single_blm, upper_to_sblm, lower_to_sblm, lower_primal_dual_map, lower_dual_to_sblm =
        build_bilevel(upper, lower, moi_link, moi_upper, mode, moi_link2,
            copy_names = model.copy_names, pass_start = model.pass_start)

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
        print_lp(single_blm, bilevel_prob, file_format)
    end

    sblm_to_solver = MOI.copy_to(solver, single_blm, copy_names = model.copy_names_to_solver)
    
    if _has_nlp_data(model.upper)
        # NLP requires an upstream jump model
        # probably is neough to have the fields:
        # np_data (YES)
        # moi_backend (YES)
        nlp_model = Model()
        nlp_model.moi_backend = solver
        nlp_model.nlp_data = model.upper.nlp_data
        # TODO assert varible index ordering
        vars_upper_orig = MOI.get(model.upper, MOI.ListOfVariableIndices())
        vars_in_solver = MOI.get(nlp_model, MOI.ListOfVariableIndices())
        for i in eachindex(vars_upper_orig) #less vars
            vi_up = vars_upper_orig[i]
            vi_sb = upper_to_sblm[vi_up]
            vi_ss = sblm_to_solver[vi_sb]
            vi_is = vars_in_solver[i]
            if vi_ss != vi_is
                error("Failed building Non linear problem, please report an issue")
                # in case jump or MOI change something in copy/nlpblock
            end
        end
        _load_nlp_data(nlp_model)
    end


    if length(solver_prob) > 0
        print_lp(solver, solver_prob, file_format)
    end

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

# Extra info

function pass_primal_info(single_blm, primal, info::VariableInfo)
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

# Bounds

function build_bounds!(::BilevelModel, ::AbstractBilevelSolverMode{T}) where T
    return nothing
end
function build_bounds!(model::BilevelModel, mode::AbstractBoundedMode{T}) where T
    return _build_bounds!(model, mode.cache)
end
function _build_bounds!(model::BilevelModel, mode::ComplementBoundCache)
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
        info.lower = max(lb, inf_if_nan(-, info.lower))
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
            info.upper = min.(ub, inf_if_nan.(+, info.upper))
            info.lower = max.(lb, inf_if_nan.(-, info.lower))
            @assert sum(info.lower .<= info.upper) == length(info.upper)
            # TODO vector
            fa_vi_ld[ctr] = info
        end
    end
    return nothing
end

inf_if_nan(::typeof(+), val) = ifelse(isnan(val), Inf, val)
inf_if_nan(::typeof(-), val) = ifelse(isnan(val), -Inf, val)

dual_lower_bound(::CI{F,LT{T}}) where {F,T} = -Inf
dual_upper_bound(::CI{F,LT{T}}) where {F,T} =  0.0

dual_lower_bound(::CI{F,GT{T}}) where {F,T} =  0.0
dual_upper_bound(::CI{F,GT{T}}) where {F,T} = +Inf

dual_lower_bound(::CI{F,ET{T}}) where {F,T} =  0.0
dual_upper_bound(::CI{F,ET{T}}) where {F,T} =  0.0

dual_lower_bound(::CI{F,S}) where {F,S} = -Inf
dual_upper_bound(::CI{F,S}) where {F,S} = +Inf

# Initialize

function _check_solver(bm::BilevelModel)
    if bm.solver === nothing
        error("No solver attached, use `set_optimizer(model, optimizer_constructor)` or initialize with `BilevelModel(optimizer_constructor)`")
    end
end

function JuMP.set_optimizer(bm::BilevelModel, optimizer_constructor;
    bridge_constraints::Bool=true, with_names=true)
    # error_if_direct_mode(model, :set_optimizer)
    if bridge_constraints
        # We set `with_names=false` because the names are handled by the first
        # caching optimizer. If `default_copy_to` without names is supported,
        # no need for a second cache.
        optimizer = MOI.instantiate(optimizer_constructor, with_bridge_type=Float64, with_names=with_names)
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


function pass_cache(bm::BilevelModel, mode::FortunyAmatMcCarlMode{T}) where T
    mode.cache = bm.mode.cache
    return nothing
end
function pass_cache(bm::BilevelModel, mode::AbstractBilevelSolverMode{T}) where T
    return nothing
end

function check_mixed_mode(::MixedMode{T}) where T
end
function check_mixed_mode(mode)
    error("Cant set/get mode on a specific object because the base mode is $mode while it should be MixedMode in this case. Run `set_mode(model, BilevelJuMP.MixedMode())`")
end
function set_mode(ci::BilevelConstraintRef, mode::AbstractBilevelSolverMode{T}) where T
    bm = ci.model
    check_mixed_mode(bm.mode)
    _mode = deepcopy(mode)
    pass_cache(bm, _mode)
    ctr = JuMP.index(bm.ctr_lower[ci.index])
    bm.mode.constraint_mode_map_c[ctr] = _mode
    return nothing
end
function unset_mode(ci::BilevelConstraintRef)
    bm = ci.model
    check_mixed_mode(bm.mode)
    ctr = JuMP.index(bm.ctr_lower[ci.index])
    delete!(bm.mode.constraint_mode_map_c, ctr)
    return nothing
end

function get_mode(ci::BilevelConstraintRef)
    bm = ci.model
    check_mixed_mode(bm.mode)
    ctr = JuMP.index(bm.ctr_lower[ci.index])
    if haskey(bm.mode.constraint_mode_map_c, ctr)
        return bm.mode.constraint_mode_map_c[ctr]
    else
        return nothing
    end
end

function set_mode(::BilevelConstraintRef, ::MixedMode{T}) where T
    error("Cant set MixedMode in a specific constraint")
end
function set_mode(::BilevelConstraintRef, ::StrongDualityMode{T}) where T
    error("Cant set StrongDualityMode in a specific constraint")
end

function set_mode(vi::BilevelVariableRef, mode::AbstractBilevelSolverMode{T}) where T
    bm = vi.model
    check_mixed_mode(bm.mode)
    _mode = deepcopy(mode)
    pass_cache(bm, _mode)
    var = JuMP.index(bm.var_lower[vi.idx])
    # var is the MOI backend index in jump
    bm.mode.constraint_mode_map_v[var] = _mode
    return nothing
end
function unset_mode(vi::BilevelVariableRef)
    bm = vi.model
    check_mixed_mode(bm.mode)
    var = JuMP.index(bm.var_lower[vi.idx])
    delete!(bm.mode.constraint_mode_map_v, var)
    return nothing
end

function get_mode(vi::BilevelVariableRef)
    bm = vi.model
    check_mixed_mode(bm.mode)
    var = JuMP.index(bm.var_lower[vi.idx])
    if haskey(bm.mode.constraint_mode_map_v, var)
        return bm.mode.constraint_mode_map_v[var]
    else
        return nothing
    end
end

function set_mode(::BilevelVariableRef, ::MixedMode{T}) where T
    error("Cant set MixedMode in a specific variable")
end
function set_mode(::BilevelVariableRef, ::StrongDualityMode{T}) where T
    error("Cant set StrongDualityMode in a specific variable")
end

function MOI.set(::BilevelModel, ::MOI.LazyConstraintCallback, func)
    error("Callbacks are not available in BilevelJuMP Models")
end
function MOI.set(::BilevelModel, ::MOI.UserCutCallback, func)
    error("Callbacks are not available in BilevelJuMP Models")
end
function MOI.set(::BilevelModel, ::MOI.HeuristicCallback, func)
    error("Callbacks are not available in BilevelJuMP Models")
end