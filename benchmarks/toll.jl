using BilevelJuMP
using Random

function bench_toll(nodes, optimizer, mode, seed = 1234)
    rng = Random.MersenneTwister(seed)

    Nodes = nodes

    # g, d = LightGraphs.euclidean_graph(Nodes, 2, seed = seed)
    # costs = Dict(k => 10*d[k]^2 for k in keys(d))

    costs = rand(rng, Nodes, Nodes) # complete graph

    cap = min(3, Nodes)

    Fares = 3

    b = zeros(Nodes)
    b[1] = -cap
    b[end] = +cap

    ctrs = []
    ctrs_e = []
    vars = []

    # MOI.empty!(optimizer)
    model = BilevelModel(optimizer; mode = mode)
    try
        JuMP.set_time_limit_sec(model, MAX_TIME)
    catch e
        @show e
        @show "failed to set limit time"
    end

    v = @variable(
        Upper(model),
        0 <= F[n = 1:Nodes, w = 1:Nodes, s = 1:Fares; n > w] <= cap
    )
    for vv in v
        push!(vars, vv)
    end

    c = @constraint(
        Upper(model),
        [n = 1:Nodes, w = 1:Nodes; n > w],
        sum(F[n, w, s] for s in 1:Fares) <= cap
    )
    for cc in c
        push!(ctrs, cc)
    end

    #=
        lower model
    =#

    v = @variable(
        Lower(model),
        0 <= f[n = 1:Nodes, w = 1:Nodes, s = 1:Fares; n > w] <= cap
    )
    for vv in v
        push!(vars, vv)
    end

    v = @variable(
        Lower(model),
        0 <= a[n = 1:Nodes, w = 1:Nodes, s = 1:Fares; n > w] <= 100
    )
    for vv in v
        push!(vars, vv)
    end

    c = @constraint(
        Lower(model),
        [n = 1:Nodes],
        sum(f[n, w, s] for w in 1:Nodes, s in 1:Fares if n > w) -
        sum(f[w, n, s] for w in 1:Nodes, s in 1:Fares if w > n) == b[n]
    )
    for cc in c
        push!(ctrs_e, cc)
    end

    c = @constraint(
        Lower(model),
        [n = 1:Nodes, w = 1:Nodes, s = 1:Fares; n > w],
        s * costs[n, w] * f[n, w, s] <= a[n, w, s]
    )
    for cc in c
        push!(ctrs, cc)
    end

    c = @constraint(
        Lower(model),
        [n = 1:Nodes, w = 1:Nodes, s = 1:Fares; n > w],
        f[n, w, s] <= F[n, w, s]
    )
    for cc in c
        push!(ctrs, cc)
    end

    @objective(
        Lower(model),
        Min,
        sum(
            costs[n, w] * f[n, w, s] + a[n, w, s] for
            n in 1:Nodes, w in 1:Nodes, s in 1:Fares if n > w
        )
    )

    #=
        upper model
    =#

    @objective(
        Upper(model),
        Max,
        sum(a[n, w, s] for n in 1:Nodes, w in 1:Nodes, s in 1:Fares if n > w)
    )

    #=
        Optimize
    =#

    optimize!(model)

    #=
    for v in vars
        val = value(v) # >= 0
    end

    for c in ctrs
        val = value(c) - normalized_rhs(c) # >= 0
    end
    =#

    @show primal_st = primal_status(model)
    @show term_st = termination_status(model) #in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED, MOI.ALMOST_LOCALLY_SOLVED]

    solve_t = JuMP.solve_time(model)
    build_t = BilevelJuMP.build_time(model)

    obj_l = try
        objective_value(Lower(model))
    catch
        NaN
    end
    obj_u = try
        objective_value(Upper(model))
    catch
        NaN
    end
    gap = try
        bound = objective_bound(Upper(model))
        abs(obj_u - bound) / max(abs(bound), 1e-8)
    catch
        NaN
    end

    return primal_st, term_st, solve_t, build_t, obj_l, obj_u, gap
end
