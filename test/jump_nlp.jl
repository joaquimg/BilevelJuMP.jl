function jump_nlp_01(
    optimizer;
    mode = BilevelJuMP.ProductMode(1e-8),
    config = Config(),
)
    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(() -> optimizer; mode = mode)
    BilevelJuMP.set_copy_names(model)

    @variable(Upper(model), x >= 0, start = 2)
    @variable(Lower(model), y >= 0, start = 0)

    @objective(Upper(model), Max, x - 3y)

    @NLconstraint(Upper(model), x^2 <= 4)

    @objective(Lower(model), Min, y)

    @constraint(Lower(model), y >= 0)

    optimize!(model)

    primal_status(model)

    @test termination_status(model) in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED]

    @test objective_value(model) ≈ 2 atol = atol

    @test value(x) ≈ 2 atol = atol
    @test value(y) ≈ 0 atol = atol
end

function jump_nlp_02(
    optimizer;
    mode = BilevelJuMP.ProductMode(1e-8),
    config = Config(),
)
    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(() -> optimizer; mode = mode)
    BilevelJuMP.set_copy_names(model)

    @variable(Upper(model), x >= 0, start = 2)
    @variable(Lower(model), y >= 0, start = 0)

    @objective(Upper(model), Max, x - 3y)

    @NLconstraint(Upper(model), x^2 <= 4)
    @NLconstraint(Upper(model), x^2 + y + 2 >= 4)
    @NLconstraint(Upper(model), x^3 + 18 * y >= 1)
    @NLconstraint(Upper(model), -1 <= x * y <= 1)

    @objective(Lower(model), Min, y)

    @constraint(Lower(model), y >= 0)

    optimize!(model)

    primal_status(model)

    @test termination_status(model) in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED]

    @test objective_value(model) ≈ 2 atol = atol

    @test value(x) ≈ 2 atol = atol
    @test value(y) ≈ 0 atol = atol
end

function jump_nlp_03(
    optimizer;
    mode = BilevelJuMP.ProductMode(1e-8),
    config = Config(),
)
    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(() -> optimizer; mode = mode)
    BilevelJuMP.set_copy_names(model)

    @variable(Upper(model), x >= 0, start = 2)
    @variable(Lower(model), y >= 0, start = 0)
    @variable(LowerOnly(model), z >= 0, start = 0)

    @objective(Upper(model), Max, x)
    # TODO NL obj
    @NLobjective(Upper(model), Max, x^2 - 3y)

    @test_throws ErrorException @NLconstraint(Lower(model), x^2 + y + 2 >= 4)
    @NLconstraint(Upper(model), x^2 <= 4)
    @test_throws ErrorException @NLconstraint(Lower(model), x^3 + 18 * y >= 1)
    @NLconstraint(Upper(model), -1 <= x * y <= 1)
    @test_throws ErrorException @NLconstraint(Upper(model), -1 <= z * y <= 1)

    @test_throws ErrorException @NLobjective(Lower(model), Min, y^3)
    @objective(Lower(model), Min, y^2 + z)

    @constraint(Lower(model), y >= 0)

    optimize!(model)

    primal_status(model)

    @test termination_status(model) in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED]

    @test objective_value(model) ≈ 4 atol = atol

    @test value(x) ≈ 2 atol = atol
    @test value(y) ≈ 0 atol = atol
end

function jump_nlp_04(
    optimizer;
    mode = BilevelJuMP.ProductMode(1e-8),
    config = Config(),
)
    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(() -> optimizer; mode = mode)
    BilevelJuMP.set_copy_names(model)

    @variable(Upper(model), x >= 0, start = 2)
    @variable(Lower(model), y >= 0, start = 0)

    @test_throws ErrorException @NLparameter(model, rhs == 4.0)
    @test_throws ErrorException @NLparameter(Lower(model), rhs == 4.0)
    @NLparameter(Upper(model), rhs == 4.0)

    @objective(Upper(model), Max, x - 3y)

    @NLconstraint(Upper(model), x^2 <= rhs)

    @objective(Lower(model), Min, y)

    @constraint(Lower(model), y >= 0)

    optimize!(model)

    primal_status(model)

    @test termination_status(model) in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED]

    @test objective_value(model) ≈ 2 atol = atol

    @test value(x) ≈ 2 atol = atol
    @test value(y) ≈ 0 atol = atol

    #=
        change param
    =#

    JuMP.set_value(rhs, 9.0)

    optimize!(model)

    primal_status(model)

    @test termination_status(model) in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED]

    @test objective_value(model) ≈ 3 atol = atol

    @test value(x) ≈ 3 atol = atol
    @test value(y) ≈ 0 atol = atol
end
