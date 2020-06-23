using Random

function bench_forecast(prods, samples, optimizer, mode, seed = 1234)

    rng = Random.MersenneTwister(seed)

    SampleSize = samples
    Products = prods

    Samples = 2:SampleSize

    phi1 = 0.7 * ones(Products)
    phi0 = 3 * ones(Products)

    mu = 3.0/(1.0 - 0.7)

    demand = ones(SampleSize, Products)

    for p in 1:Products
        for t in Samples
            demand[t, p] = max(0.0, phi0[p] + phi1[p] * demand[t-1, p] + 0.2 * randn(rng))
        end
    end

    p_s = 5 .+ 0.1*collect(1:Products)
    p_b = 4 .+ 0.1*collect(1:Products)
    p_r = 3 .+ 0.1*collect(1:Products)

    u = 8*Products

    ctrs = []
    vars = []

    model = BilevelModel()

    # parameters
    v = @variable(Upper(model),  0 <= φ0[1:Products] <= 10)
    push!(vars, vec(v))
    v = @variable(Upper(model), -1 <= φ1[1:Products] <= 1)
    push!(vars, vec(v))

    #=
        lower model
    =#

    # buying quantity is the only
    v = @variable(Lower(model), 0 <= q_b[p=1:Products, t=Samples] <= u)
    push!(vars, vec(v))

    #
    v = @variable(Lower(model), 0 <= q_s[p=1:Products, t=Samples] <= u)
    push!(vars, vec(v))
    v = @variable(Lower(model), 0 <= q_r[p=1:Products, t=Samples] <= u)
    push!(vars, vec(v))


    c = @constraint(Lower(model),[p=1:Products,t=Samples],
        q_s[p,t] + q_r[p,t] <= q_b[p,t])
    push!(ctrs, vec(c))

    c = @constraint(Lower(model),[p=1:Products,t=Samples],
        q_s[p,t] <= φ0[p] + φ1[p] * demand[t-1, p])
    push!(ctrs, vec(c))

    c = @constraint(Lower(model),[t=Samples],
        sum(q_b[p,t] for p in 1:Products) <= u)
    push!(ctrs, vec(c))

    @objective(Lower(model), Min,
        sum(p_b[p] * q_b[p,t] + p_s[p] * q_s[p,t] + p_r[p] * q_r[p,t]
            for t in Samples, p in 1:Products))

    #=
        upper model
    =#

    #
    v = @variable(Upper(model), 0 <= q2_s[p=1:Products, t=Samples] <= u)
    push!(vars, vec(v))
    v = @variable(Upper(model), 0 <= q2_r[p=1:Products, t=Samples] <= u)
    push!(vars, vec(v))

    c = @constraint(Upper(model),[p=1:Products,t=Samples],
        q2_s[p,t] + q2_r[p,t] <= q_b[p,t])
    push!(ctrs, vec(c))

    c = @constraint(Upper(model),[p=1:Products,t=Samples],
        q2_s[p,t] <= demand[t, p])
    push!(ctrs, vec(c))

    @objective(Upper(model), Min,
        sum(p_b[p] * q_b[p,t] + p_s[p] * q2_s[p,t] + p_r[p] * q2_r[p,t]
            for t in Samples, p in 1:Products))

    #=
        Optimize
    =#

    MOI.empty!(optimizer)
    optimize!(model, optimizer, mode)

    @show primal_status(model)

    @show termination_status(model) #in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED, MOI.ALMOST_LOCALLY_SOLVED]

    #=
    for v in vars
        val = value(v) # >= 0
    end

    for c in ctrs
        val = value(c) - normalized_rhs(c) # >= 0
    end
    =#
end