"""
    MixedMode(; default = SOS1Mode())

A mode that allows to mix different modes for different constraints and variables.

* `default` is the default mode to use for all constraints and variables that are
  not explicitly mapped to a mode.
"""
mutable struct MixedMode{T} <: AbstractBilevelSolverMode{T}
    default::AbstractBilevelSolverMode{T}
    constraint_mode_map_c::Dict{CI,AbstractBilevelSolverMode{T}}
    constraint_mode_map_v::Dict{VI,AbstractBilevelSolverMode{T}}
    cache::ComplementBoundCache
    function_cache::Dict{Int,MOI.AbstractScalarFunction}
    function MixedMode(; default = SOS1Mode())
        return new{Float64}(
            default,
            Dict{CI,AbstractBilevelSolverMode{Float64}}(),
            Dict{VI,AbstractBilevelSolverMode{Float64}}(),
            ComplementBoundCache(),
            Dict{Int,MOI.AbstractScalarFunction}(),
        )
    end
end

function reset!(mode::MixedMode)
    mode.cache = ComplementBoundCache()
    mode.function_cache = Dict{Int,MOI.AbstractScalarFunction}()
    for v in values(mode.constraint_mode_map_c)
        reset!(v)
    end
    for v in values(mode.constraint_mode_map_v)
        reset!(v)
    end
    return nothing
end

function build_full_map!(
    mode::MixedMode,
    upper_idxmap,
    lower_idxmap,
    lower_dual_idxmap,
    lower_primal_dual_map,
)
    _build_bound_map!(
        mode.cache,
        upper_idxmap,
        lower_idxmap,
        lower_dual_idxmap,
        lower_primal_dual_map,
    )
    return nothing
end

accept_vector_set(::MixedMode{T}, ::Complement) where {T} = nothing

function add_aggregate_constraints(m, mode::MixedMode{T}, copy_names) where {T}
    if isempty(mode.function_cache)
        return nothing
    end
    eps = Dict{Int,T}()
    for list in (
        mode.constraint_mode_map_c,
        mode.constraint_mode_map_v,
        Dict(nothing => mode.default),
    )
        for md in values(list)
            val, idx = _get_eps_idx(md)
            if haskey(eps, idx) && idx > 0
                eps[idx] = max(eps[idx], val)
            else #idx >= 0
                eps[idx] = val
            end
        end
    end
    for (idx, func) in mode.function_cache
        _add_aggregate_constraints(m, func, eps[idx], idx, copy_names)
    end
    return nothing
end

function add_complement(
    mode::MixedMode{T},
    m,
    comp::Complement,
    idxmap_primal,
    idxmap_dual,
    copy_names::Bool,
    pass_start::Bool,
) where {T}
    _mode = get_mode(mode, comp.constraint, idxmap_primal)
    accept_vector_set(_mode, comp)
    ret = add_complement(
        _mode,
        m,
        comp,
        idxmap_primal,
        idxmap_dual,
        copy_names,
        pass_start,
    )
    add_function_to_cache(mode, _mode)
    return ret
end
add_function_to_cache(mode::MixedMode{T}, _mode) where {T} = nothing
function add_function_to_cache(
    mode::MixedMode{T},
    _mode::ProductMode{T},
) where {T}
    idx = _mode.aggregation_group
    if _mode.function_cache !== nothing && idx > 0
        if haskey(mode.function_cache, idx)
            mode.function_cache[idx] = MOIU.operate(
                +,
                T,
                mode.function_cache[idx],
                _mode.function_cache,
            )
        else
            mode.function_cache[idx] = _mode.function_cache
        end
        _mode.function_cache = nothing
    end
    return nothing
end

function get_mode(
    mode::MixedMode{T},
    ci::CI{F,S},
    map,
) where {
    T,
    F<:MOI.VariableIndex,
    S<:Union{MOI.EqualTo{T},MOI.LessThan{T},MOI.GreaterThan{T}},
}
    # key = map[VI(ci.value)]
    key = VI(ci.value)
    if haskey(mode.constraint_mode_map_v, key)
        return mode.constraint_mode_map_v[key]
    else
        return mode.default
    end
end
function get_mode(mode::MixedMode{T}, ci::CI{F,S}, map) where {T,F,S}
    key = ci
    if haskey(mode.constraint_mode_map_c, key)
        return mode.constraint_mode_map_c[key]
    else
        return mode.default
    end
end

function _get_eps_idx(mode)
    return 0.0, 0
end

function _get_eps_idx(mode::ProductMode{T}) where {T}
    idx = mode.aggregation_group
    eps = mode.epsilon
    return eps, idx
end