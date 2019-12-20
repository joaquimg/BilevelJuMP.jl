const SVF = MOI.SingleVariable
const VVF = MOI.VectorOfVariables
const SAF{T} = MOI.ScalarAffineFunction{T}
const VAF{T} = MOI.VectorAffineFunction{T}

const VI = MOI.VariableIndex
const CI = MOI.ConstraintIndex

# abstract type AsbtractBilevelOptimizer end
# struct SOS1Optimizer{O} <: AsbtractBilevelOptimizer
#     solver::O
#     # options
# end
# function SOS1Optimizer(solver::O) where O
#     return SOS1Optimizer{O}(solver)
# end

struct Complement#{M1 <: MOI.ModelLike, M2 <: MOI.ModelLike, F, S}
    # primal::M1
    func_w_cte#::F
    set_w_zero#::S
    # dual::M2
    variable#::VI
end

abstract type BilevelSolverMode{T} end

mutable struct SOS1Mode{T} <: BilevelSolverMode{T}
    epsilon::T
    function SOS1Mode()
        return new{Float64}(zero(Float64))
    end
end

mutable struct ComplementMode{T} <: BilevelSolverMode{T}
    epsilon::T
    function ComplementMode()
        return new{Float64}(zero(Float64))
    end
end

mutable struct ComplementWithSlackMode{T} <: BilevelSolverMode{T}
    epsilon::T
    function ComplementWithSlackMode()
        return new{Float64}(zero(Float64))
    end
end

mutable struct ProductMode{T} <: BilevelSolverMode{T}
    epsilon::T
    function ProductMode()
        return new{Float64}(zero(Float64))
    end
end

mutable struct ProductWithSlackMode{T} <: BilevelSolverMode{T}
    epsilon::T
    function ProductWithSlackMode()
        return new{Float64}(zero(Float64))
    end
end

function get_canonical_complements(primal_model, primal_dual_map)
    map = primal_dual_map.primal_con_dual_var
    out = Complement[]
    T = Float64
    for ci in keys(map)
        func = MOI.get(primal_model, MOI.ConstraintFunction(), ci)
        set = MOI.get(primal_model, MOI.ConstraintSet(), ci)
        i = 1
        func.constant = Dualization.set_dot(i, set, T)*Dualization.get_scalar_term(primal_model, i, ci)
        # todo - set dot on function
        con = Complement(func, set_with_zero(set), map[ci][1])
        push!(out, con)
    end
    return out
end

function set_with_zero(set::Union{MOI.LessThan{T},
    MOI.GreaterThan{T}, MOI.EqualTo{T}}) where T
    return typeof(set)(0.0)
end
function set_with_zero(set)
    return copy(set)
end

function build_bilevel(
    upper::MOI.ModelLike, lower::MOI.ModelLike,
    link::Dict{VI,VI}, upper_variables::Vector{VI},
    mode)

    # Start with an empty problem
    moi_mode = MOIU.AUTOMATIC
    m = MOIU.CachingOptimizer(MOIU.Model{Float64}(), moi_mode)

    # add the first level
    copy_names = false
    # key are from src, value are from dest
    upper_idxmap = MOIU.default_copy_to(m, upper, copy_names)
    pass_names(m, upper, upper_idxmap)

    # append the second level primal
    lower_idxmap = MOIU.IndexMap() #

    for (upper_key, lower_val) in link
        lower_idxmap[lower_val] = upper_idxmap[upper_key]
    end

    append_to(m, lower, lower_idxmap, copy_names)
    pass_names(m, lower, lower_idxmap)


    # dualize the second level
    dual_problem = dualize(lower,
        dual_names = DualNames("dual_","dual_"),
        variable_parameters = upper_variables,
        ignore_objective = true)
    lower_dual = dual_problem.dual_model
    lower_primal_dual_map = dual_problem.primal_dual_map

    # appende the second level dual
    lower_dual_idxmap = MOIU.IndexMap()

    append_to(m, lower_dual, lower_dual_idxmap, copy_names)
    pass_names(m, lower_dual, lower_dual_idxmap)

    # complete KKT
    # 1 - primal dual equality (quadratic equality constraint)
    #     use quadratic or log expansion (requires bounds on variables get from outside or from prob)
    # 1a - no slacks
    # 1b - use slack
    # 2 - complementarity
    # 2a - actual complementarity constraints w and w/o slack (ok)
    # 2b - SOS1 (slack * duals) (ok)
    # 2c - Big-M formulation (Fortuny-Amat and McCarl, 1981)
    # 2d - Product of variables  w and w/o slack (ok)
    # 3 - NLP (y = argmin_y problem)

    # futurely add slacks to primal model
    comps = get_canonical_complements(lower, lower_primal_dual_map)
    for comp in comps
        add_complement(mode, m, comp, lower_idxmap, lower_dual_idxmap)
    end

    return m, upper_idxmap, lower_idxmap#, lower_primal_dual_map, lower_dual_idxmap
end

function add_complement(mode::ComplementMode{T}, m, comp::Complement, idxmap_primal, idxmap_dual) where T
    f = comp.func_w_cte
    s = comp.set_w_zero
    v = comp.variable

    f_dest = MOIU.map_indices.(Ref(idxmap_primal), f)

    dual = idxmap_dual[v]

    new_f = MOIU.operate(vcat, T, f_dest, MOI.SingleVariable(dual))

    c1 = MOI.add_constraint(m, 
        new_f,
        MOI.Complements(1))

    nm = MOI.get(m, MOI.VariableName(), dual)
    MOI.set(m, MOI.ConstraintName(), c1, "compl_compl_($(nm))")

    return c1
end

function add_complement(mode::ComplementWithSlackMode{T}, m, comp::Complement, idxmap_primal, idxmap_dual) where T
    f = comp.func_w_cte
    s = comp.set_w_zero
    v = comp.variable

    slack, slack_in_set = MOI.add_constrained_variable(m, s)
    f_dest = MOIU.map_indices.(Ref(idxmap_primal), f)
    new_f = MOIU.operate(-, T, f_dest, MOI.SingleVariable(slack))
    equality = MOIU.normalize_and_add_constraint(m, new_f, MOI.EqualTo(zero(T)))

    dual = idxmap_dual[v]
    c1 = MOI.add_constraint(m, 
        MOI.VectorOfVariables([slack, dual]),
        MOI.Complements(1))

    nm = MOI.get(m, MOI.VariableName(), dual)
    MOI.set(m, MOI.VariableName(), slack, "slk_($(nm))")
    MOI.set(m, MOI.ConstraintName(), slack_in_set, "bound_slk_($(nm))")
    MOI.set(m, MOI.ConstraintName(), equality, "eq_slk_($(nm))")
    MOI.set(m, MOI.ConstraintName(), c1, "compl_complWslk_($(nm))")

    return slack, slack_in_set, equality, c1
end

function add_complement(mode::SOS1Mode{T}, m, comp::Complement, idxmap_primal, idxmap_dual) where T
    f = comp.func_w_cte
    s = comp.set_w_zero
    v = comp.variable

    slack, slack_in_set = MOI.add_constrained_variable(m, s)
    f_dest = MOIU.map_indices.(Ref(idxmap_primal), f)
    new_f = MOIU.operate(-, T, f_dest, MOI.SingleVariable(slack))
    equality = MOIU.normalize_and_add_constraint(m, new_f, MOI.EqualTo(zero(T)))

    dual = idxmap_dual[v]
    c1 = MOI.add_constraint(m, 
        MOI.VectorOfVariables([slack, dual]),
        MOI.SOS1([1.0, 2.0]))

    nm = MOI.get(m, MOI.VariableName(), dual)
    MOI.set(m, MOI.VariableName(), slack, "slk_($(nm))")
    MOI.set(m, MOI.ConstraintName(), slack_in_set, "bound_slk_($(nm))")
    MOI.set(m, MOI.ConstraintName(), equality, "eq_slk_($(nm))")
    MOI.set(m, MOI.ConstraintName(), c1, "compl_sos1_($(nm))")

    return slack, slack_in_set, equality, c1
end

function add_complement(mode::ProductMode{T}, m, comp::Complement, idxmap_primal, idxmap_dual) where T
    f = comp.func_w_cte
    s = comp.set_w_zero
    v = comp.variable

    f_dest = MOIU.map_indices.(Ref(idxmap_primal), f)

    dual = idxmap_dual[v]

    new_f = MOIU.operate(*, T, f_dest, MOI.SingleVariable(dual))

    c1 = MOI.add_constraint(m, 
        new_f,
        MOI.EqualTo(zero(T)))

    nm = MOI.get(m, MOI.VariableName(), dual)
    MOI.set(m, MOI.ConstraintName(), c1, "compl_prod_($(nm))")

    return c1
end

function add_complement(mode::ProductWithSlackMode{T}, m, comp::Complement, idxmap_primal, idxmap_dual) where T
    f = comp.func_w_cte
    s = comp.set_w_zero
    v = comp.variable

    slack, slack_in_set = MOI.add_constrained_variable(m, s)
    f_dest = MOIU.map_indices.(Ref(idxmap_primal), f)
    new_f = MOIU.operate(-, T, f_dest, MOI.SingleVariable(slack))
    equality = MOIU.normalize_and_add_constraint(m, new_f, MOI.EqualTo(zero(T)))

    dual = idxmap_dual[v]

    prod_f = MOIU.operate(*, T, MOI.SingleVariable(slack), MOI.SingleVariable(dual))

    c1 = MOI.add_constraint(m, 
        prod_f,
        MOI.EqualTo(zero(T)))

    nm = MOI.get(m, MOI.VariableName(), dual)
    MOI.set(m, MOI.VariableName(), slack, "slk_($(nm))")
    MOI.set(m, MOI.ConstraintName(), slack_in_set, "bound_slk_($(nm))")
    MOI.set(m, MOI.ConstraintName(), equality, "eq_slk_($(nm))")
    MOI.set(m, MOI.ConstraintName(), c1, "compl_prodWslk_($(nm))")

    return slack, slack_in_set, equality, c1
end

function pass_names(dest, src, map)
    for vi in MOI.get(src, MOI.ListOfVariableIndices())
        name = MOI.get(src, MOI.VariableName(), vi)
        MOI.set(dest, MOI.VariableName(), map[vi], name)
    end
    for (F,S) in MOI.get(src, MOI.ListOfConstraints())
        for con in MOI.get(src, MOI.ListOfConstraintIndices{F,S}())
            name = MOI.get(src, MOI.ConstraintName(), con)
            MOI.set(dest, MOI.ConstraintName(), map[con], name)
        end
    end
end

function append_to(dest::MOI.ModelLike, src::MOI.ModelLike, idxmap, copy_names::Bool)
    # MOI.empty!(dest)

    # idxmap = MOIU.IndexMap()

    vis_src = MOI.get(src, MOI.ListOfVariableIndices())
    constraint_types = MOI.get(src, MOI.ListOfConstraints())
    single_variable_types = [S for (F, S) in constraint_types
                             if F == MOI.SingleVariable]
    vector_of_variables_types = [S for (F, S) in constraint_types
                                 if F == MOI.VectorOfVariables]

    # The `NLPBlock` assumes that the order of variables does not change (#849)
    if MOI.NLPBlock() in MOI.get(src, MOI.ListOfModelAttributesSet())
        error("not nlp for now")
        vector_of_variables_not_added = [
            MOI.get(src, MOI.ListOfConstraintIndices{MOI.VectorOfVariables, S}())
            for S in vector_of_variables_types
        ]
        single_variable_not_added = [
            MOI.get(src, MOI.ListOfConstraintIndices{MOI.SingleVariable, S}())
            for S in single_variable_types
        ]
    else
        vector_of_variables_not_added = [
            MOIU.copy_vector_of_variables(dest, src, idxmap, S)
            for S in vector_of_variables_types
        ]
        single_variable_not_added = [
            MOIU.copy_single_variable(dest, src, idxmap, S)
            for S in single_variable_types
        ]
    end

    MOIU.copy_free_variables(dest, idxmap, vis_src, MOI.add_variables)

    # Copy variable attributes
    # MOIU.pass_attributes(dest, src, copy_names, idxmap, vis_src)

    # Copy model attributes
    # pass_attributes(dest, src, copy_names, idxmap)

    # Copy constraints
    MOIU.pass_constraints(dest, src, copy_names, idxmap,
                     single_variable_types, single_variable_not_added,
                     vector_of_variables_types, vector_of_variables_not_added)

    return idxmap
end