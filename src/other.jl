
function split_variables(saf::MOI.ScalarAffineFunction{T},
    variable_parameters::Vector{VI}) where T
    remove = get_indices_variables(saf, variable_parameters)
    new_saf = MOI.ScalarAffineFunction{T}(saf.terms[remove], 0.0)
    old_saf = copy(saf)
    deleteat!(old_saf, remove)
    return old_saf, new_saf
end

function get_parametric_constant(saf::MOI.ScalarAffineFunction{T},
    variable_parameters::Vector{VI}) where T
    terms = get_indices_variables(saf, variable_parameters)
    MOI.ScalarAffineFunction{T}(saf.terms[terms], 0.0)
end

function get_parametric_constants(primal_model::MOI.ModelLike,
    # primal_dual_map::PrimalDualMap{T}, dual_names::DualNames,
    con_types::Vector{Tuple{DataType, DataType}}, variable_parameters::Vector{VI}) where T
    # todo
    param_functions = Dict()
    for (F, S) in con_types
        for ci in MOI.get(primal_model, MOI.ListOfConstraintIndices{F,S}()) # Constraints of type {F, S}
            func = MOI.get(primal_model, MOI.ConstraintFunction(), ci)
            set = MOI.get(primal_model, MOI.ConstraintSet(), ci)
            i = 1 # for scalar affine
            val = set_dot(i, set, T)*get_scalar_term(primal_model, i, ci)

            # todo - requires a setdot here
            param_functions[ci] = get_parametric_constant(func, variable_parameters)


            param_functions[ci].constant = val
        end
    end
    return param_functions
end

function get_canonical_functions(primal_model::MOI.ModelLike)
    # todo
    T = Float64
    con_types = MOI.get(primal_model, MOI.ListOfConstraints())
    functions = Dict()
    for (F, S) in con_types
        for ci in MOI.get(primal_model, MOI.ListOfConstraintIndices{F,S}()) # Constraints of type {F, S}
            func = MOI.get(primal_model, MOI.ConstraintFunction(), ci)
            set = MOI.get(primal_model, MOI.ConstraintSet(), ci)
            i = 1 # for scalar affine
            func.constant = set_dot(i, set, T)*get_scalar_term(primal_model, i, ci)

            # todo - requires a setdot here
            functions[ci] = func
        end
    end
    return functions
end