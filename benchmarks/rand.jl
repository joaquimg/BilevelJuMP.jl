using SparseArrays, LinearAlgebra
using BilevelJuMP
using Random

function bench_rand(rows, cols, density, optimizer, mode, seed = 1234)

    # seed = 1234
    # rows = 5
    # cols = 5
    # density = 0.4

    rng = Random.MersenneTwister(seed)

    f_A(r, n) = ifelse.(rand(r, n) .> 0.25, 1, -1) .* (15 .+ 30 .* rand(r, n))

    Al = sprand(rng, rows, cols, density, f_A)
    Au = sprand(rng, rows, cols, density, f_A)

    Bl = sprand(rng, rows, cols, density, f_A)
    Bu = sprand(rng, rows, cols, density, f_A)

    bl = 50 * rand(rng, rows)
    bu = 50 * rand(rng, rows)

    s_l = diagm(0 => ifelse.(rand(rng, rows) .> 0.2, 1, -1))
    s_u = diagm(0 => ifelse.(rand(rng, rows) .> 0.2, 1, -1))

    f_c(r, n) = 20 .* 2 .* (rand(r, n) .- 0.5)

    cl = sprand(rng, cols, 0.5, f_c)
    cu = sprand(rng, cols, 0.5, f_c)

    # MOI.empty!(optimizer)
    model = BilevelModel(optimizer; mode = mode)
    try
        JuMP.set_time_limit_sec(model, MAX_TIME)
    catch e
        @show e
        @show "failed to set limit time"
    end
    @variable(Upper(model), -1000 <= x[1:cols] <= 1000)
    @variable(Lower(model), -1000 <= y[1:cols] <= 1000)

    @constraint(Upper(model), (s_u * Au) * x .+ (s_u * Bu) * y .<= s_u * bu)
    @constraint(Lower(model), (s_l * Al) * x .+ (s_l * Bl) * y .<= s_l * bl)

    @objective(Upper(model), Min, cu' * x)
    @objective(Lower(model), Min, cl' * y)

    #=
        Optimize
    =#

    optimize!(model)

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
