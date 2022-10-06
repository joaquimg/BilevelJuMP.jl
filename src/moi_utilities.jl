
function pass_names(dest, src, map)
    for vi in MOI.get(src, MOI.ListOfVariableIndices())
        name = MOI.get(src, MOI.VariableName(), vi)
        if name != ""
            MOI.set(dest, MOI.VariableName(), map[vi], name)
        end
    end
    for (F, S) in MOI.get(src, MOI.ListOfConstraintTypesPresent())
        if !(F <: MOI.VariableIndex)
            for con in MOI.get(src, MOI.ListOfConstraintIndices{F,S}())
                name = MOI.get(src, MOI.ConstraintName(), con)
                if name != ""
                    MOI.set(dest, MOI.ConstraintName(), map[con], name)
                end
            end
        end
    end
end

function append_to(
    dest::MOI.ModelLike,
    src::MOI.ModelLike,
    idxmap,
    filter_constraints::Union{Nothing,Function} = nothing;
    allow_single_bounds::Bool = true,
)

    #=
        This function follows closely the function `default_copy_to` defined in
        MathOptInterface.Utilities
        due to some caveats of this function we keep the commented functions
        from the original function to highlight the differences and 
        easen the burden of updating when `default_copy_to` is updated.
    =#

    # MOI.empty!(dest)

    # idxmap = MOIU.IndexMap()

    vis_src = MOI.get(src, MOI.ListOfVariableIndices())
    # index_map_for_variable_indices only initializes the data structure
    # idxmap = index_map_for_variable_indices(vis_src)

    # The `NLPBlock` assumes that the order of variables does not change (#849)
    if MOI.NLPBlock() in MOI.get(src, MOI.ListOfModelAttributesSet())
        error("NLP models are not supported.")
        # constraint_types = MOI.get(src, MOI.ListOfConstraints())
        # single_variable_types = [S for (F, S) in constraint_types
        #                          if F == MOI.VariableIndex]
        # vector_of_variables_types = [S for (F, S) in constraint_types
        #                              if F == MOI.VectorOfVariables]
        # vector_of_variables_not_added = [
        #     MOI.get(src, MOI.ListOfConstraintIndices{MOI.VectorOfVariables, S}())
        #     for S in vector_of_variables_types
        # ]
        # single_variable_not_added = [
        #     MOI.get(src, MOI.ListOfConstraintIndices{MOI.VariableIndex, S}())
        #     for S in single_variable_types
        # ]
    else
        # the key asusmption here is that MOI keeps the following behaviour
        # "The copy is only done when
        # the variables to be copied are not already keys of `idxmap`. It returns a list
        # of the constraints copied and not copied."
        # from copy_single_variable and copy_vector_of_variables.
        # this is very importante because variables are shered between
        # upper, lower and lower dual levels
        constraints_not_added = Any[
            MOIU._try_constrain_variables_on_creation(dest, src, idxmap, S)
            for S in MOIU.sorted_variable_sets_by_cost(dest, src)
        ]
    end

    # MOIU.copy_free_variables(dest, idxmap, vis_src, MOI.add_variables)
    # copy variables has a size check that dows not generalizes here
    # because we have previously added variables
    for vi in vis_src
        if !haskey(idxmap.var_map, vi)
            var = MOI.add_variable(dest)
            idxmap.var_map[vi] = var
        end
    end

    # Copy variable attributes
    MOIU.pass_attributes(dest, src, idxmap, vis_src)

    # Copy model attributes
    # attention HERE to no pass objective functions!
    # pass_attributes(dest, src, copy_names, idxmap)

    # Copy constraints
    MOIU._pass_constraints(dest, src, idxmap, constraints_not_added)

    return idxmap
end

# scalar
function MOIU.promote_operation(
    ::typeof(LinearAlgebra.dot),
    ::Type{T},
    ::Type{<:Union{MOI.VariableIndex,MOI.ScalarAffineFunction{T}}},
    ::Type{T},
) where {T}
    return MOI.ScalarAffineFunction{T}
end
function MOIU.promote_operation(
    ::typeof(LinearAlgebra.dot),
    ::Type{T},
    ::Type{T},
    ::Type{<:Union{MOI.VariableIndex,MOI.ScalarAffineFunction{T}}},
) where {T}
    return MOI.ScalarAffineFunction{T}
end
function MOIU.promote_operation(
    ::typeof(LinearAlgebra.dot),
    ::Type{T},
    ::Type{<:Union{MOI.VariableIndex,MOI.ScalarAffineFunction{T}}},
    ::Type{<:Union{MOI.VariableIndex,MOI.ScalarAffineFunction{T}}},
) where {T}
    return MOI.ScalarQuadraticFunction{T}
end
function MOIU.promote_operation(
    ::typeof(LinearAlgebra.dot),
    ::Type{T},
    ::Type{MOI.ScalarQuadraticFunction{T}},
    ::Type{T},
) where {T}
    return MOI.ScalarQuadraticFunction{T}
end
function MOIU.promote_operation(
    ::typeof(LinearAlgebra.dot),
    ::Type{T},
    ::Type{T},
    ::Type{MOI.ScalarQuadraticFunction{T}},
) where {T}
    return MOI.ScalarQuadraticFunction{T}
end
# flip
function MOIU.operate(
    ::typeof(LinearAlgebra.dot),
    ::Type{T},
    f::Union{
        MOI.VariableIndex,
        MOI.ScalarAffineFunction{T},
        MOI.ScalarQuadraticFunction{T},
    },
    α::T,
) where {T}
    return MOIU.operate(LinearAlgebra.dot, T, α, f)
end
# pass to *
function MOIU.operate(
    ::typeof(LinearAlgebra.dot),
    ::Type{T},
    f::Union{T,MOI.VariableIndex,MOI.ScalarAffineFunction{T}},
    g::Union{MOI.VariableIndex,MOI.ScalarAffineFunction{T}},
) where {T}
    return MOIU.operate(*, T, f, g)
end
function MOIU.operate(
    ::typeof(LinearAlgebra.dot),
    ::Type{T},
    α::T,
    f::MOI.ScalarQuadraticFunction{T},
) where {T}
    return MOIU.operate(*, T, f, α)
end

# vector
function MOIU.promote_operation(
    ::typeof(LinearAlgebra.dot),
    ::Type{T},
    ::Type{<:Union{MOI.VectorOfVariables,MOI.VectorAffineFunction{T}}},
    ::Type{Vector{T}},
) where {T}
    return MOI.VectorAffineFunction{T}
end
function MOIU.promote_operation(
    ::typeof(LinearAlgebra.dot),
    ::Type{T},
    ::Type{Vector{T}},
    ::Type{<:Union{MOI.VectorOfVariables,MOI.VectorAffineFunction{T}}},
) where {T}
    return MOI.VectorAffineFunction{T}
end
function MOIU.promote_operation(
    ::typeof(LinearAlgebra.dot),
    ::Type{T},
    ::Type{<:Union{MOI.VectorOfVariables,MOI.VectorAffineFunction{T}}},
    ::Type{<:Union{MOI.VectorOfVariables,MOI.VectorAffineFunction{T}}},
) where {T}
    return MOI.VectorQuadraticFunction{T}
end
function MOIU.promote_operation(
    ::typeof(LinearAlgebra.dot),
    ::Type{T},
    ::Type{MOI.VectorQuadraticFunction{T}},
    ::Type{Vector{T}},
) where {T}
    return MOI.VectorQuadraticFunction{T}
end
function MOIU.promote_operation(
    ::typeof(LinearAlgebra.dot),
    ::Type{T},
    ::Type{Vector{T}},
    ::Type{MOI.VectorQuadraticFunction{T}},
) where {T}
    return MOI.VectorQuadraticFunction{T}
end
# flip
function MOIU.operate(
    ::typeof(LinearAlgebra.dot),
    ::Type{T},
    f::Union{
        MOI.VectorOfVariables,
        MOI.VectorAffineFunction{T},
        MOI.VectorQuadraticFunction{T},
    },
    α::Vector{T},
) where {T}
    return MOIU.operate(LinearAlgebra.dot, T, α, f)
end
# pass to _operate(LinearAlgebra.dot, ...)
function MOIU.operate(
    ::typeof(LinearAlgebra.dot),
    ::Type{T},
    f::Union{Vector{T},MOI.VectorOfVariables,MOI.VectorAffineFunction{T}},
    g::Union{MOI.VectorOfVariables,MOI.VectorAffineFunction{T}},
) where {T}
    return _operate(LinearAlgebra.dot, T, f, g)
end
function MOIU.operate(
    ::typeof(LinearAlgebra.dot),
    ::Type{T},
    α::T,
    f::MOI.VectorQuadraticFunction{T},
) where {T}
    return _operate(LinearAlgebra.dot, T, f, α)
end
function _operate(
    ::typeof(LinearAlgebra.dot),
    ::Type{T},
    f::Union{
        Vector{T},
        MOI.VectorOfVariables,
        MOI.VectorAffineFunction{T},
        MOI.VectorQuadraticFunction{T},
    },
    g::Union{
        MOI.VectorOfVariables,
        MOI.VectorAffineFunction{T},
        MOI.VectorQuadraticFunction{T},
    },
) where {T}
    dim = MOI.output_dimension(g)
    if MOI.output_dimension(f) != dim
        throw(
            DimensionMismatch(
                "f and g are of different MOI.output_dimension's!",
            ),
        )
    end

    fs = MOIU.scalarize(f)
    gs = MOIU.scalarize(g)

    out = MOIU.operate(*, T, fs[1], gs[1])
    for i in 2:dim
        MOIU.operate!(+, T, out, MOIU.operate(*, T, fs[i], gs[i]))
    end

    return out
end
MOIU.scalarize(v::Vector{T}) where {T<:Number} = v
MOI.output_dimension(v::Vector{T}) where {T<:Number} = length(v)#
