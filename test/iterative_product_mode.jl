function iterative_product_mode_01()
    # This example is from the book Decomposition Techniques in Mathematical Programming, as used in the examples section
    # Chapter 7.2, page 281, [url](https://www.springer.com/gp/book/9783540276852)
    iter_eps = [1e-0 * 0.95^i for i = 1:300]

    iter_attr = Dict(
        "mu_init" => 1e-9,
        "warm_start_init_point" => "yes",
        "warm_start_bound_push" => 1e-12,
        "warm_start_bound_frac" => 1e-12,
        "warm_start_slack_bound_frac" => 1e-12,
        "warm_start_slack_bound_push" => 1e-12,
        "warm_start_mult_bound_push" => 1e-12,
        # "output_file" => "solver.log",
        # "file_print_level" => 8,
        "print_level" => 0,
    )

    model = BilevelModel(
        Ipopt.Optimizer,
        mode = BilevelJuMP.ProductMode(1e-0; iter_eps = iter_eps, iter_attr = iter_attr),
    )

    @variable(Upper(model), x)
    @variable(UpperOnly(model), z)
    @variable(Lower(model), y)
    @variable(LowerOnly(model), w)
    @objective(Upper(model), Min, -x + 4y + z)
    @constraint(Upper(model), y + 2x + z <= 9)
    @constraint(Upper(model), z == 1)
    @objective(Lower(model), Min, -x - y + w)
    @constraint(Lower(model), y >= 0)
    @constraint(Lower(model), x + y + w <= 8)
    @constraint(Lower(model), x >= 0)
    @constraint(Lower(model), x <= 4)
    @constraint(Lower(model), w == 1)


    set_optimizer_attribute(model, "mu_target", 1e-9)

    optimize!(model)

    @test value(x) ≈ 1 atol = 1e-4
    @test value(y) ≈ 6 atol = 1e-4
    @test value(z) ≈ 1 atol = 1e-4
    @test value(w) ≈ 1 atol = 1e-4

end

function iterative_product_mode_02_1()
    # This example is from the paper S. Siddiqui & S. A. Gabriel (2012): "An SOS1-Based Approach for Solving MPECs with a Natural Gas Market Application", as appearing in "Networks and Spatial Economics"
    # It can be found in section 3 "Numerical Examples"
    # Dataset 1

    F = [1, 2]
    c = Dict(1 => 1, 2 => 1)
    C = 1
    a = 13
    b = 1

    iter_eps = [1e-0 * 0.95^i for i = 1:300]

    iter_attr = Dict(
        "mu_init" => 1e-9,
        "warm_start_init_point" => "yes",
        "warm_start_bound_push" => 1e-12,
        "warm_start_bound_frac" => 1e-12,
        "warm_start_slack_bound_frac" => 1e-12,
        "warm_start_slack_bound_push" => 1e-12,
        "warm_start_mult_bound_push" => 1e-12,
        # "output_file" => "solver.log",
        # "file_print_level" => 8,
        "print_level" => 0,
    )

    model1 = BilevelModel(
        Ipopt.Optimizer,
        mode = BilevelJuMP.ProductMode(1e-0; iter_eps = iter_eps, iter_attr = iter_attr),
    )

    @variable(Lower(model1), q[F] >= 0)
    @variable(Upper(model1), Q >= 0)

    @objective(Upper(model1), Max, ((a - b * (q[1] + q[2] + Q)) * Q - C * Q))

    @objective(
        Lower(model1),
        Min,
        -(
            (a - b * (q[1] + q[2] + Q)) * q[1] - C * q[1] +
            (a - b * (q[1] + q[2] + Q)) * q[2] - C * q[2] + b * q[1] * q[2]
        )
    )

    set_optimizer_attribute(model1, "mu_target", 1e-9)

    optimize!(model1)

    @test isapprox(value(model1[:Q]), 6; atol = 1e-6)
    @test isapprox(value.(model1[:q]).data, [2, 2]; atol = 1e-6)

end

function iterative_product_mode_02_2()
    # This example is from the paper S. Siddiqui & S. A. Gabriel (2012): "An SOS1-Based Approach for Solving MPECs with a Natural Gas Market Application", as appearing in "Networks and Spatial Economics" 
    # It can be found in section 3 "Numerical Examples"
    # Dataset 2: 

    F = [1, 2]
    c = Dict(1 => 1, 2 => 1)
    C = 1
    a = 13
    b = 0.1

    iter_eps = [1e-0 * 0.95^i for i = 1:300]

    iter_attr = Dict(
        "mu_init" => 1e-9,
        "warm_start_init_point" => "yes",
        "warm_start_bound_push" => 1e-12,
        "warm_start_bound_frac" => 1e-12,
        "warm_start_slack_bound_frac" => 1e-12,
        "warm_start_slack_bound_push" => 1e-12,
        "warm_start_mult_bound_push" => 1e-12,
        "print_level" => 0,
    )

    model2 = BilevelModel(
        Ipopt.Optimizer,
        mode = BilevelJuMP.ProductMode(1e-0; iter_eps = iter_eps, iter_attr = iter_attr),
    )

    @variable(Lower(model2), q[F] >= 0)
    @variable(Upper(model2), Q >= 0)

    @objective(Upper(model2), Max, ((a - b * (q[1] + q[2] + Q)) * Q - C * Q))

    @objective(
        Lower(model2),
        Min,
        -(
            (a - b * (q[1] + q[2] + Q)) * q[1] - C * q[1] +
            (a - b * (q[1] + q[2] + Q)) * q[2] - C * q[2] + b * q[1] * q[2]
        )
    )

    set_optimizer_attribute(model2, "mu_target", 1e-9)

    optimize!(model2)

    # Auto testing

    @test isapprox(value(model2[:Q]), 60; atol = 1e-6)
    @test isapprox(value.(model2[:q]).data, [20, 20]; atol = 1e-6)

end

function iterative_product_mode_02_3()
    # This example is from the paper S. Siddiqui & S. A. Gabriel (2012): "An SOS1-Based Approach for Solving MPECs with a Natural Gas Market Application", as appearing in "Networks and Spatial Economics"
    # It can be found in section 3 "Numerical Examples"
    # Dataset 3:

    F = [1, 2]
    c = Dict(1 => 1, 2 => 1)
    C = 2
    a = 13
    b = 0.1

    iter_eps = [1e-0 * 0.95^i for i = 1:300]

    iter_attr = Dict(
        "mu_init" => 1e-9,
        "warm_start_init_point" => "yes",
        "warm_start_bound_push" => 1e-12,
        "warm_start_bound_frac" => 1e-12,
        "warm_start_slack_bound_frac" => 1e-12,
        "warm_start_slack_bound_push" => 1e-12,
        "warm_start_mult_bound_push" => 1e-12,
        "print_level" => 0,
    )

    model3 = BilevelModel(
        Ipopt.Optimizer,
        mode = BilevelJuMP.ProductMode(1e-0; iter_eps = iter_eps, iter_attr = iter_attr),
    )

    @variable(Lower(model3), q[F] >= 0)
    @variable(Upper(model3), Q >= 0)

    @objective(Upper(model3), Max, ((a - b * (q[1] + q[2] + Q)) * Q - C * Q))

    @objective(
        Lower(model3),
        Min,
        -(
            (a - b * (q[1] + q[2] + Q)) * q[1] - C * q[1] +
            (a - b * (q[1] + q[2] + Q)) * q[2] - C * q[2] + b * q[1] * q[2]
        )
    )

    set_optimizer_attribute(model3, "mu_target", 1e-9)

    optimize!(model3)

    # Auto testing

    @test isapprox(value(model3[:Q]), 55; atol = 1e-6)
    @test isapprox(value.(model3[:q]).data, [18.333, 18.333]; atol = 1e-2)

end
