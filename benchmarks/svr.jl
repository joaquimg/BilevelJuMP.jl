function bench_svr(dim, sample, optimizer, mode, seed = 1234)

    rng = Random.MersenneTwister(seed)

    # Toll Setting
    # SVR
    # Forecasting
    # Random

    # Generate SVR sample
    Dimension = dim#2
    SampleSize = sample#20
    InSampleFrac = 0.5

    x = 1 * 2 * (rand(rng, SampleSize, Dimension) .- 0.5)
    w_real = ones(Dimension)
    y = x * w_real .+ 0.1 * 2 * (rand(rng, SampleSize, Dimension) .- 0.5)

    split_point = ceil(Int, SampleSize * InSampleFrac)
    InSample = 1:split_point
    OutSample = (split_point+1):SampleSize

    ctrs = []
    vars = []

    model = BilevelModel()

    # hyper parameters
    @variable(Upper(model), C >= 0)
    @variable(Upper(model), ε >= 0)

    # classifier (support vector)
    @variable(Lower(model), w[1:Dimension])

    #=
        Upper level
    =#

    # absolute value extra variables
    @variable(Upper(model), a_up_pos[i=OutSample] >= 0)
    @variable(Upper(model), a_up_neg[i=OutSample] >= 0)

    for i in OutSample
        c = @constraint(Upper(model),
            a_up_pos[i] >= + sum(w[j]*x[i,j] for j in 1:Dimension) - y[i])
        push!(ctrs, c)
        c = @constraint(Upper(model),
            a_up_neg[i] >= - sum(w[j]*x[i,j] for j in 1:Dimension) + y[i])
        push!(ctrs, c)
    end

    @objective(Upper(model),
        Min, sum(a_up_pos[i] + a_up_neg[i] for i in OutSample))

    #=
        Lower level
    =#

    @variable(Lower(model), a_lo_pos[i=InSample] >= 0)
    @variable(Lower(model), a_lo_neg[i=InSample] >= 0)

    @variable(Lower(model), a_lo_max[i=InSample] >= 0)

    for i in InSample
        c = @constraint(Lower(model),
            a_lo_pos[i] >= + sum(w[j]*x[i,j] for j in 1:Dimension) - y[i])
        push!(ctrs, c)
        c = @constraint(Lower(model),
            a_lo_neg[i] >= - sum(w[j]*x[i,j] for j in 1:Dimension) + y[i])
        push!(ctrs, c)
        c = @constraint(Lower(model),
            a_lo_max[i] >= a_lo_pos[i] + a_lo_neg[i] - ε)
        push!(ctrs, c)
    end

    @objective(Lower(model),
        Min, C * sum(a_lo_max[i] for i in InSample) + 
            1/2*sum(w[j]^2 for j in 1:Dimension))

    #=
        Optimize
    =#

    MOI.empty!(optimizer)
    optimize!(model, optimizer, mode)

    @show primal_status(model)

    @show termination_status(model) #in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED, MOI.ALMOST_LOCALLY_SOLVED]

    # stack bounded vars
    push!(vars, C)
    push!(vars, ε)
    for i in InSample
        push!(vars, a_lo_pos[i])
        push!(vars, a_lo_neg[i])
        push!(vars, a_lo_max[i])
    end
    for i in OutSample
        push!(vars, a_up_pos[i])
        push!(vars, a_up_neg[i])
    end

    #=
    for v in vars
        val = value(v) # >= 0
    end

    for c in ctrs
        val = value(c) - normalized_rhs(c) # >= 0
    end
    =#
end