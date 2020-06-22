using SparseArrays
using LinearAlgebra

function bench_rand(rows, cols, density, optimizer, mode, seed = 1234)

    seed = 1234
    rows = 5
    cols = 5
    density = 0.4

    rng = Random.MersenneTwister(seed)

    f_A(r, n) = ifelse.(rand(r, n) .> 0.75, 1, -1) .* (15 .+ 30 .* rand(r, n))

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

    model = BilevelModel()

    @variable(Upper(model), -100 <= x[1:cols] <= 100)
    @variable(Lower(model), -100 <= y[1:cols] <= 100)

    @constraint(Upper(model), (s_u * Au) * x .+ (s_u * Bu) * y .<= s_u * bu)
    @constraint(Lower(model), (s_l * Al) * x .+ (s_l * Bl) * y .<= s_l * bl)

    @objective(Upper(model), Min, cu' * x)
    @objective(Upper(model), Min, cl' * y)


    #=
        Optimize
    =#

    MOI.empty!(optimizer)
    optimize!(model, optimizer, mode)

    @show primal_status(model)

    @show termination_status(model)



end
