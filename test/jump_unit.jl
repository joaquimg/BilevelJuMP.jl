function jump_objective()

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -4x -3y)

    @objective(Lower(model), Min, y)

    @constraints(Lower(model), begin
        c1, 2x+y <= 4
        c2, x+2y <= 4
        c3, x >= 0
        c4, y >= 0
    end)

    tp = JuMP.objective_function_type(Lower(model))
    JuMP.objective_function(Lower(model), tp)

    @test JuMP.objective_sense(model) == MOI.MIN_SENSE
    @test_throws ErrorException JuMP.relative_gap(model)
    @test_throws ErrorException JuMP.dual_objective_value(model)
    @test_throws ErrorException JuMP.objective_bound(model)
    @test_throws ErrorException JuMP.set_objective(model, MOI.MAX_SENSE, x)

end

function jump_objective_solver(optimizer, mode)

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -4x -3y)

    @constraints(Upper(model), begin
        cx, x == 0
    end)

    @objective(Lower(model), Min, y)

    @constraints(Lower(model), begin
        cy, y == 0
    end)

    optimize!(model)

    @test JuMP.objective_sense(model) == MOI.MIN_SENSE
    # not accepted by cbc
    # @test JuMP.relative_gap(Upper(model)) ≈ 0.0 atol=1e-3
    # @test JuMP.dual_objective_value(model) ≈ 0.0 atol=1e-3
    @test JuMP.objective_value(Upper(model)) ≈ 0.0 atol=1e-3
    JuMP.objective_bound(Upper(model))
    JuMP.set_objective(Upper(model), MOI.MAX_SENSE, x)

end

function jump_display()
    atol = config.atol

    # config.bound_hint = true

    # min -4x -3y
    # s.t.
    # y = argmin_y y
    #      2x + y <= 4
    #       x +2y <= 4
    #       x     >= 0
    #           y >= 0
    #
    # sol: x = 2, y = 0
    # obj_upper = -8
    # obj_lower =  0

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -4x -3y)

    @objective(Lower(model), Min, y)

    @constraints(Lower(model), begin
        c1, 2x+y <= 4
        c2, x+2y <= 4
        c3, x >= 0
        c4, y >= 0
    end)

    @test JuMP.num_constraints(model) == 
        JuMP.num_constraints(Upper(model)) + JuMP.num_constraints(Lower(model)) 

    display(x)
    println()
    display(c2)
    println()
    display(model)
    println()
    display(Upper(model))
    println()
    display(Lower(model))
    println()

    xx  = JuMP.variable_by_name(model, "x")
    cc2 = JuMP.constraint_by_name(model, "c2")

    # set_optimizer(model, MOIU.Model{Float64})
    # display(model)
    # println()

end

function invalid_lower_objective(optimizer, mode)
    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -4x -3y)

    @test_throws ErrorException optimize!(model)
    return
end

function invalid_optimizer(optimizer, mode)
    @test_throws ErrorException BilevelModel(optimizer, mode = mode)
    return
end

function jump_display_solver(optimizer, mode)
    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    # config.bound_hint = true

    # min -4x -3y
    # s.t.
    # y = argmin_y y
    #      2x + y <= 4

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -4x -3y)

    @objective(Lower(model), Min, y)

    @constraints(Lower(model), begin
        c1, 2x+y <= 4
    end)

    display(model)
    println()
    display(Upper(model))
    println()
    display(Lower(model))
    println()

end

function jump_bounds()
    atol = config.atol

    model = BilevelModel()

    # config.bound_hint = true

    # min -4x -3y
    # s.t.
    # y = argmin_y y
    #      2x + y <= 4

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -4x -3y)

    @objective(Lower(model), Min, y)

    @constraints(Lower(model), begin
        c, 2x+y <= 4
    end)

    @variable(Upper(model), l, DualOf(c))

    for var in [x, y, l]
        @test has_lower_bound(var) == false
        set_lower_bound(var, 12)
        @test lower_bound(var) == 12
        @test has_lower_bound(var) == true
        delete_lower_bound(var)
        @test has_lower_bound(var) == false
    end

    for var in [x, y, l]
        @test has_upper_bound(var) == false
        set_upper_bound(var, 12)
        @test upper_bound(var) == 12
        @test has_upper_bound(var) == true
        delete_upper_bound(var)
        @test has_upper_bound(var) == false
    end

    for var in [x, y]
        @test is_fixed(var) == false
        fix(var, 12)
        @test fix_value(var) == 12
        @test is_fixed(var) == true
        unfix(var)
        @test is_fixed(var) == false
    end

    @test is_fixed(l) == false
    @test_throws ErrorException fix(l, 12)
    @test_throws ErrorException unfix(l)

    for var in [x, y]
        set_start_value(var, 12)
        @test start_value(var) == 12
        set_start_value(var, 13)
        @test start_value(var) == 13
    end

    for var in [x, y]
        @test is_binary(var) == false
        set_binary(var)
        @test is_binary(var) == true
        unset_binary(var)
        @test is_binary(var) == false
    end

    @test is_binary(l) == false
    @test_throws ErrorException set_binary(l)
    @test_throws ErrorException unset_binary(l)

    for var in [x, y]
        @test is_integer(var) == false
        set_integer(var)
        @test is_integer(var) == true
        unset_integer(var)
        @test is_integer(var) == false
    end

    @test is_integer(l) == false
    @test_throws ErrorException set_integer(l)
    @test_throws ErrorException unset_integer(l)

end

function jump_attributes()

    model = BilevelModel()

    # min -4x -3y
    # s.t.
    # y = argmin_y y
    #      2x + y <= 4

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -4x -3y)

    @objective(Lower(model), Min, y)

    @constraints(Lower(model), begin
        c, 2x+y <= 4
    end)

    BilevelJuMP.set_copy_names(model)
    @test BilevelJuMP.get_copy_names(model)
    BilevelJuMP.unset_copy_names(model)
    @test !BilevelJuMP.get_copy_names(model)

    BilevelJuMP.set_pass_start(model)
    @test BilevelJuMP.get_pass_start(model)
    BilevelJuMP.unset_pass_start(model)
    @test !BilevelJuMP.get_pass_start(model)

    @test isnan(JuMP.solve_time(model))
    @test isnan(BilevelJuMP.build_time(model))

    @test_throws MethodError JuMP.set_optimizer_attributes(mode, "weird" => true, "strange" => "yes")

    return nothing
end

function jump_attributes_solver(optimizer, mode)

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -4x -3y)

    @constraints(Upper(model), begin
        cx, x == 0
    end)

    @objective(Lower(model), Min, y)

    @constraints(Lower(model), begin
        cy, y == 0
    end)

    optimize!(model)

    @test JuMP.objective_sense(model) == MOI.MIN_SENSE
    # not accepted by cbc
    # @test JuMP.relative_gap(Upper(model)) ≈ 0.0 atol=1e-3
    # @test JuMP.dual_objective_value(model) ≈ 0.0 atol=1e-3
    @test JuMP.objective_value(Upper(model)) ≈ 0.0 atol=1e-3
    JuMP.objective_bound(Upper(model))
    JuMP.set_objective(Upper(model), MOI.MAX_SENSE, x)

    JuMP.solve_time(model)
    BilevelJuMP.build_time(model)

    JuMP.set_optimizer_attribute(model, MOI.Silent(), true)
    JuMP.unset_silent(model)
    JuMP.set_silent(model)

    JuMP.set_time_limit_sec(model, 3.0)
    @test JuMP.time_limit_sec(model) == 3.0
    JuMP.unset_time_limit_sec(model)

    @test JuMP.result_count(model) == 1
    JuMP.node_count(model)
    @test_throws ArgumentError JuMP.simplex_iterations(model)
    @test_throws ArgumentError JuMP.barrier_iterations(model)

end