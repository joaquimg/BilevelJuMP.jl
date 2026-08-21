# Copyright (c) 2019: Joaquim Dias Garcia, and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

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

# Add every variable and constraint of `src` to `dest`, extending `idxmap` with
# the new indices, and return `idxmap`.
#
# Unlike `MOI.Utilities.default_copy_to`, `dest` is not emptied and `idxmap` is
# expected to come in partially filled: variables that appear in more than one
# level are added once and then shared, so the entries already in `idxmap` are
# reused instead of creating duplicates. Model attributes are deliberately NOT
# copied, the objective function in particular, since `dest` holds the objective
# of the upper level.
#
# The steps are kept in the same order as `default_copy_to`, down to which
# variables are constrained on creation: the order in which variables reach the
# solver changes its iterates, and the tests do check numbers that a nonlinear
# solver only gets right to a tolerance.
function _append_to(dest::MOI.ModelLike, src::MOI.ModelLike, idxmap)
    # The `NLPBlock` assumes that the order of variables does not change (#849)
    if MOI.NLPBlock() in MOI.get(src, MOI.ListOfModelAttributesSet())
        error("NLP models are not supported.")
    end
    # Variables that carry a `VariableIndex`/`VectorOfVariables` constraint are
    # added together with it, and so come first. Whatever is left over - both
    # the free variables and the constraints that could not be added this way -
    # is added below.
    not_added = Any[
        _add_constrained_variables(dest, src, idxmap, S) for
        S in MOIU.sorted_variable_sets_by_cost(dest, src)
    ]
    vis_src = MOI.get(src, MOI.ListOfVariableIndices())
    for vi in vis_src
        if !haskey(idxmap, vi)
            idxmap[vi] = MOI.add_variable(dest)
        end
    end
    MOIU.pass_attributes(dest, src, idxmap, vis_src)
    # Model attributes are deliberately NOT copied, the objective function in
    # particular, since `dest` holds the objective of the upper level.
    #
    # What follows is `MOI.Utilities._pass_constraints`.
    for cis_src in not_added
        _copy_constraints(dest, src, idxmap, cis_src)
    end
    constraint_types = MOI.get(src, MOI.ListOfConstraintTypesPresent())
    nonvariable_constraint_types = filter(constraint_types) do (F, S)
        return !(F <: Union{MOI.VariableIndex,MOI.VectorOfVariables})
    end
    MOIU.pass_nonvariable_constraints(
        dest,
        src,
        idxmap,
        nonvariable_constraint_types,
    )
    for (F, S) in constraint_types
        cis_src = MOI.get(src, MOI.ListOfConstraintIndices{F,S}())
        MOIU.pass_attributes(dest, src, idxmap, cis_src)
    end
    return idxmap
end

# Add the variables of the `VariableIndex`-in-`S` constraints of `src` to `dest`
# with `MOI.add_constrained_variable`, and return the constraints that could not
# be added that way because their variable is shared with another level and is
# therefore already in `idxmap`.
function _add_constrained_variables(
    dest::MOI.ModelLike,
    src::MOI.ModelLike,
    idxmap,
    ::Type{S},
) where {S<:MOI.AbstractScalarSet}
    F = MOI.VariableIndex
    not_added = MOI.ConstraintIndex{F,S}[]
    for ci in MOI.get(src, MOI.ListOfConstraintIndices{F,S}())
        func = MOI.get(src, MOI.ConstraintFunction(), ci)
        if haskey(idxmap, func)
            push!(not_added, ci)
        else
            set = MOI.get(src, MOI.ConstraintSet(), ci)::S
            vi_dest, ci_dest = MOI.add_constrained_variable(dest, set)
            idxmap[ci] = ci_dest
            idxmap[func] = vi_dest
        end
    end
    return not_added
end

# Same as above for `VectorOfVariables`-in-`S`. A constraint whose function
# repeats a variable cannot be added on creation either.
function _add_constrained_variables(
    dest::MOI.ModelLike,
    src::MOI.ModelLike,
    idxmap,
    ::Type{S},
) where {S<:MOI.AbstractVectorSet}
    F = MOI.VectorOfVariables
    not_added = MOI.ConstraintIndex{F,S}[]
    for ci in MOI.get(src, MOI.ListOfConstraintIndices{F,S}())
        func = MOI.get(src, MOI.ConstraintFunction(), ci)
        if !allunique(func.variables) ||
           any(vi -> haskey(idxmap, vi), func.variables)
            push!(not_added, ci)
        else
            set = MOI.get(src, MOI.ConstraintSet(), ci)::S
            vis_dest, ci_dest = MOI.add_constrained_variables(dest, set)
            idxmap[ci] = ci_dest
            for (vi_src, vi_dest) in zip(func.variables, vis_dest)
                idxmap[vi_src] = vi_dest
            end
        end
    end
    return not_added
end

# Copy the constraints `cis_src` of `src` over to `dest`, without their
# attributes. This mirrors the private `MOI.Utilities._copy_constraints`.
function _copy_constraints(
    dest::MOI.ModelLike,
    src::MOI.ModelLike,
    idxmap,
    cis_src,
)
    for ci in cis_src
        func = MOI.get(src, MOI.ConstraintFunction(), ci)
        set = MOI.get(src, MOI.ConstraintSet(), ci)
        func = MOIU.map_indices(idxmap, func)
        idxmap[ci] = MOI.add_constraint(dest, func, set)
    end
    return nothing
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
