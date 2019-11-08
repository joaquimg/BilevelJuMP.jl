const SVF = MOI.SingleVariable
const VVF = MOI.VectorOfVariables
const SAF{T} = MOI.ScalarAffineFunction{T}
const VAF{T} = MOI.VectorAffineFunction{T}

const VI = MOI.VariableIndex
const CI = MOI.ConstraintIndex

struct Complement#{M1 <: MOI.ModelLike, M2 <: MOI.ModelLike, F, S}
    # primal::M1
    func#::F
    set#::S
    # dual::M2
    variable#::VI
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

function build_bilivel(upper::MOI.ModelLike, lower::MOI.ModelLike,
                       link::Dict{VI,VI}, upper_variables::Vector{VI})

    # Start with an empty problem
    mode = MOIU.AUTOMATIC
    m = MOIU.CachingOptimizer(MOIU.Model{Float64}(), mode)

    # add the first level
    copy_names = false
    # key are from src, value are from dest
    upper_idxmap = MOIU.default_copy_to(m, upper, copy_names)
    pass_names(m, upper, upper_idxmap)

    # append the second level primal
    lower_idxmap = MOIU.IndexMap() #

    for i in keys(upper_idxmap.varmap)
        lower_idxmap[link[i]] = upper_idxmap[i]
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
    # 1a - no slacks
    # 1b - use slack
    # 2 - complementarity
    # 2a - actual complementarity constraints
    # 2b - SOS1 (slack * duals)
    # 2c - Big-M formulation (Fortuny-Amat and McCarl, 1981)
    # 3 - NLP (y = argmin_y problem)

    if true # SOS1
        # futurely add slacks to primal model
        comps = get_canonical_complements(lower, lower_primal_dual_map)
        for comp in comps
            # create a constrained slack - depends on set
            add_complement_constraint(m, comp, lower_idxmap, lower_dual_idxmap)
            # add directly to the primal equality
            # (will require set change - loses dual mapping)
        end

    end

    return m, upper_idxmap, lower_idxmap#, lower_primal_dual_map, lower_dual_idxmap
end

function add_complement_constraint(m, comp::Complement, idxmap_primal, idxmap_dual)
    f = comp.func
    s = comp.set
    v = comp.variable
    T = Float64
    slack, slack_in_set = MOI.add_constrained_variable(m, s)
    f_dest = MOIU.map_indices.(Ref(idxmap_primal), f)
    new_f = MOIU.operate(-, T, f_dest, MOI.SingleVariable(slack))
    equality = MOIU.normalize_and_add_constraint(m, new_f, MOI.EqualTo(zero(T)))

    dual = idxmap_dual[v]
    c1 = MOI.add_constraint(m, 
        MOI.VectorOfVariables([slack, dual]),
        MOI.SOS1([1.0, 2.0]))

    nm = MOI.get(m, MOI.VariableName(), dual)
    MOI.set(m, MOI.VariableName(), slack, "slk($(nm))")
    MOI.set(m, MOI.ConstraintName(), c1, "c_slk($(nm))")
    MOI.set(m, MOI.ConstraintName(), slack_in_set, "b_slk($(nm))")

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