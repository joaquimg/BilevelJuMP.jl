function jump_objective()
    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    ex1 = -4x - 3y

    @objective(Upper(model), Min, ex1)

    ex2 = y

    @objective(Lower(model), Min, ex2)

    @constraints(Lower(model), begin
        c1, 2x + y <= 4
        c2, x + 2y <= 4
        c3, x >= 0
        c4, y >= 0
    end)

    tp = JuMP.objective_function_type(Lower(model))
    @test JuMP.objective_function(Lower(model), tp) == ex2

    tp = JuMP.objective_function_type(Upper(model))
    @test JuMP.objective_function(Upper(model), tp) == ex1

    @test JuMP.objective_sense(model) == MOI.MIN_SENSE

    @test_throws ErrorException JuMP.relative_gap(model)
    @test_throws ErrorException JuMP.dual_objective_value(model)
    @test_throws ErrorException JuMP.objective_bound(model)
    @test_throws ErrorException JuMP.set_objective(model, MOI.MAX_SENSE, x)

    @objective(Lower(model), Min, 0)
    tp = JuMP.objective_function_type(Lower(model))
    @test JuMP.objective_function(Lower(model), tp) == 0

    @objective(Upper(model), Min, 0.0)
    tp = JuMP.objective_function_type(Upper(model))
    @test JuMP.objective_function(Upper(model), tp) == 0

    @test_throws ErrorException JuMP.objective_function_type(model)
    @test_throws ErrorException JuMP.objective_function(model)
    @test_throws ErrorException JuMP.objective_function(
        model,
        MOI.VariableIndex,
    )

    @test_throws ErrorException JuMP.optimize!(Upper(model))
end

function jump_constraints()
    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -4x - 3y)

    @constraints(Upper(model), begin
        cup, 2x + y <= 4
    end)

    @objective(Lower(model), Min, y)

    @constraints(Lower(model), begin
        c1, 2x + y <= 4
        c2, x + 2y <= 4
        c3, x >= 0
        c4, y >= 0
    end)

    @test JuMP.normalized_rhs(c1) == 4.0

    @test is_valid(model, x)
    @test is_valid(Upper(model), x)
    @test is_valid(Lower(model), x) # it is in both levels

    @test is_valid(model, c1)
    @test !is_valid(Upper(model), c1)
    @test is_valid(Lower(model), c1)

    # JuMP.constraint_object(c1, MOI.ScalarAffineFunction{Float64}, MOI.LessThan{Float64})

    JuMP.set_dual_start_value(c1, 1.2)
    JuMP.dual_start_value(c1) == 1.2

    BilevelJuMP.set_primal_upper_bound_hint(x, 1.7)
    @test BilevelJuMP.get_primal_upper_bound_hint(x) == 1.7
    BilevelJuMP.set_primal_lower_bound_hint(x, 1.8)
    @test BilevelJuMP.get_primal_lower_bound_hint(x) == 1.8

    @variable(Upper(model), 0 <= alpha <= 10, BilevelJuMP.DualOf(c1))
    @test_throws ErrorException @variable(
        Upper(model),
        0 <= alpha <= 10,
        BilevelJuMP.DualOf(cup)
    )
    @test_throws ErrorException @variable(
        Upper(model),
        0 <= alpha <= 10,
        BilevelJuMP.DualOf(cup),
        bad_key_arg = false
    )
    @test_throws ErrorException @constraint(model, x + 2y <= 4)
end

function jump_variables()
    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -4x - 3y)

    @constraints(Upper(model), begin
        cup, 2x + y <= 4
    end)

    @objective(Lower(model), Min, y)

    @constraints(Lower(model), begin
        c1, 2x + y <= 4
        c2, x + 2y <= 4
        c3, x >= 0
        c4, y >= 0
    end)

    @variable(Upper(model), 0 <= alpha <= 10, BilevelJuMP.DualOf(c1))

    JuMP.set_start_value(alpha, 2.2)
    @test JuMP.start_value(alpha) == 2.2

    @test x === copy(x)
    @test JuMP.isequal_canonical(x, copy(x))
    @test !JuMP.isequal_canonical(x, y)

    @test JuMP.is_valid(model, x)
end

function jump_objective_solver(optimizer, mode)
    MOI.empty!(optimizer)
    model = BilevelModel(() -> optimizer; mode = mode)

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -4x - 3y)

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
    @test JuMP.objective_value(Upper(model)) ≈ 0.0 atol = 1e-3
    JuMP.objective_bound(Upper(model))
    return JuMP.set_objective(Upper(model), MOI.MAX_SENSE, x)
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
    @variable(LowerOnly(model), z)

    @constraint(Lower(model), c0, x >= -1)

    @objective(Upper(model), Min, -4x - 3y)

    @objective(Lower(model), Min, y)

    @constraints(Lower(model), begin
        c1, 2x + y <= 4
        c2, x + 2y <= 4
        c3, x >= 0
        c4, y >= 0
    end)

    @test JuMP.num_constraints(model) ==
          JuMP.num_constraints(Upper(model)) +
          JuMP.num_constraints(Lower(model))

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

    @test JuMP.variable_by_name(model, "x") == x
    @test JuMP.variable_by_name(model, "z") == z
    @test JuMP.constraint_by_name(model, "c2") == c2
    @test JuMP.constraint_by_name(model, "c0") == c0

    # set_optimizer(model, MOIU.Model{Float64})
    # display(model)
    # println()

end

function invalid_lower_objective(optimizer, mode)
    MOI.empty!(optimizer)
    model = BilevelModel(() -> optimizer; mode = mode)

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -4x - 3y)

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
    model = BilevelModel(() -> optimizer; mode = mode)

    # config.bound_hint = true

    # min -4x -3y
    # s.t.
    # y = argmin_y y
    #      2x + y <= 4

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -4x - 3y)

    @objective(Lower(model), Min, y)

    @constraints(Lower(model), begin
        c1, 2x + y <= 4
    end)

    display(model)
    println()
    display(Upper(model))
    println()
    display(Lower(model))
    return println()
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

    @objective(Upper(model), Min, -4x - 3y)

    @objective(Lower(model), Min, y)

    @constraints(Lower(model), begin
        c, 2x + y <= 4
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

    @objective(Upper(model), Min, -4x - 3y)

    @objective(Lower(model), Min, y)

    @constraints(Lower(model), begin
        c, 2x + y <= 4
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

    @test_throws MethodError JuMP.set_optimizer_attributes(
        model,
        "weird" => true,
        "strange" => "yes",
    )
    @test_throws ErrorException JuMP.get_optimizer_attribute(model, "weird")

    return nothing
end

function jump_attributes_solver(optimizer, mode)
    MOI.empty!(optimizer)
    model = BilevelModel(() -> optimizer; mode = mode)

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -4x - 3y)

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
    @test JuMP.objective_value(Upper(model)) ≈ 0.0 atol = 1e-3
    JuMP.objective_bound(Upper(model))
    JuMP.set_objective(Upper(model), MOI.MAX_SENSE, x)

    JuMP.solve_time(model)
    BilevelJuMP.build_time(model)

    silent_mode = JuMP.get_optimizer_attribute(model, MOI.Silent())
    JuMP.set_optimizer_attribute(model, MOI.Silent(), true)
    JuMP.unset_silent(model)
    @test !JuMP.get_optimizer_attribute(model, MOI.Silent())
    JuMP.set_silent(model)
    @test JuMP.get_optimizer_attribute(model, MOI.Silent())
    JuMP.set_optimizer_attribute(model, MOI.Silent(), silent_mode)

    JuMP.set_time_limit_sec(model, 3.0)
    @test JuMP.time_limit_sec(model) == 3.0
    JuMP.unset_time_limit_sec(model)

    @test JuMP.result_count(model) == 1
    JuMP.node_count(model)
    # TODO improve this check
    @test JuMP.simplex_iterations(model) >= 0
    @test_throws Exception JuMP.barrier_iterations(model)
    # @test_throws MathOptInterface.GetAttributeNotAllowed{MathOptInterface.BarrierIterations} JuMP.barrier_iterations(model)

    @test_throws MethodError JuMP.set_optimizer_attributes(
        mode,
        "weird" => true,
        "strange" => "yes",
    )
end

function mixed_mode_unit()

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

    @variable(Upper(model), x >= 0)
    @variable(Lower(model), y >= 0)

    @objective(Upper(model), Min, -4x - 3y)

    @objective(Lower(model), Min, y)

    @constraints(Lower(model), begin
        c1, 2x + y <= 4
        c2, x + 2y <= 4
    end)

    @test_throws ErrorException BilevelJuMP.set_mode(c1, BilevelJuMP.SOS1Mode())
    @test_throws ErrorException BilevelJuMP.set_mode(
        c1,
        BilevelJuMP.IndicatorMode(),
    )
    @test_throws ErrorException BilevelJuMP.set_mode(x, BilevelJuMP.SOS1Mode())
    @test_throws ErrorException BilevelJuMP.set_mode(
        x,
        BilevelJuMP.IndicatorMode(),
    )

    BilevelJuMP.set_mode(model, BilevelJuMP.MixedMode())

    @test_throws ErrorException BilevelJuMP.set_mode(
        c1,
        BilevelJuMP.MixedMode(),
    )
    @test_throws ErrorException BilevelJuMP.set_mode(
        c1,
        BilevelJuMP.StrongDualityMode(),
    )
    @test_throws ErrorException BilevelJuMP.set_mode(x, BilevelJuMP.MixedMode())
    @test_throws ErrorException BilevelJuMP.set_mode(
        x,
        BilevelJuMP.StrongDualityMode(),
    )

    BilevelJuMP.set_mode(x, BilevelJuMP.FortunyAmatMcCarlMode())
    BilevelJuMP.set_mode(y, BilevelJuMP.IndicatorMode())
    BilevelJuMP.set_mode(c1, BilevelJuMP.FortunyAmatMcCarlMode())
    BilevelJuMP.set_mode(c2, BilevelJuMP.IndicatorMode())

    @test typeof(BilevelJuMP.get_mode(y)) <: BilevelJuMP.IndicatorMode
    @test typeof(BilevelJuMP.get_mode(c2)) <: BilevelJuMP.IndicatorMode

    BilevelJuMP.unset_mode(x)
    BilevelJuMP.unset_mode(y)
    BilevelJuMP.unset_mode(c1)
    BilevelJuMP.unset_mode(c2)

    @test BilevelJuMP.get_mode(y) === nothing
    @test BilevelJuMP.get_mode(c2) === nothing

    return nothing
end

function variables_unit()
    model = BilevelModel()

    @variable(Upper(model), w >= 0)
    @variable(Upper(model), x >= 0)
    @variable(Lower(model), y >= 0)
    @variable(Lower(model), z >= 0)

    @test Set(JuMP.all_variables(Upper(model))) == Set([x, y, z, w])
    @test Set(JuMP.all_variables(Lower(model))) == Set([x, y, z, w])
    @test Set(JuMP.all_variables(model)) == Set([x, y, z, w])

    JuMP.delete(model, x)
    JuMP.delete(model, y)

    @test Set(JuMP.all_variables(Upper(model))) == Set([w, z])
    @test Set(JuMP.all_variables(Lower(model))) == Set([w, z])
    @test Set(JuMP.all_variables(model)) == Set([w, z])

    ex = @expression(model, w + z)
    @constraint(Upper(model), ctr, ex >= 0)

    @test 0.0 == JuMP.normalized_rhs(ctr)
    JuMP.set_normalized_rhs(ctr, 4)
    @test 4.0 == JuMP.normalized_rhs(ctr)
    JuMP.add_to_function_constant(ctr, 2)
    @test 2.0 == JuMP.normalized_rhs(ctr)

    @test 1.0 == JuMP.normalized_coefficient(ctr, w)
    JuMP.set_normalized_coefficient(ctr, w, 2.0)
    @test 2.0 == JuMP.normalized_coefficient(ctr, w)

    ex1 = BilevelAffExpr(-1.0)
    add_to_expression!(ex1, 2.0, w)
    add_to_expression!(ex1, 1.0, z)
    @constraint(Lower(model), ex1 >= 0)

    ex2 = w + z + 1
    @constraint(Lower(model), ex2 >= 0)

    ex3 = ex = w^2 + 2 * w * z + z^2 + w + z - 1
    @objective(Lower(model), Min, ex3)

    @test coefficient(ex3, w, z) == 2

    return nothing
end

function jump_no_cb()
    model = BilevelModel()

    @variable(Upper(model), x >= 0)

    @test_throws ErrorException MOI.set(
        model,
        MOI.LazyConstraintCallback(),
        x -> x,
    )
    @test_throws ErrorException MOI.set(model, MOI.UserCutCallback(), x -> x)
    @test_throws ErrorException MOI.set(model, MOI.HeuristicCallback(), x -> x)

    return nothing
end

function constraint_unit()
    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @constraint(Upper(model), ctru, x == 0)
    @constraint(Lower(model), ctrl, y == 0)

    for (f, s) in JuMP.list_of_constraint_types(Upper(model))
        @test ctru == JuMP.all_constraints(Upper(model), f, s)[]
    end
    for (f, s) in JuMP.list_of_constraint_types(Lower(model))
        @test ctrl == JuMP.all_constraints(Lower(model), f, s)[]
        @test Set([ctrl, ctru]) == Set(JuMP.all_constraints(model, f, s))
    end
    @test JuMP.list_of_constraint_types(Lower(model)) ==
          JuMP.list_of_constraint_types(Upper(model))
    @test JuMP.list_of_constraint_types(Lower(model)) ==
          JuMP.list_of_constraint_types(model)

    for (f, s) in JuMP.list_of_constraint_types(Upper(model))
        @test JuMP.num_constraints(Upper(model), f, s) == 1
    end
    for (f, s) in JuMP.list_of_constraint_types(Lower(model))
        @test JuMP.num_constraints(Lower(model), f, s) == 1
    end
    JuMP.delete(model, ctru)
    @test isempty(JuMP.list_of_constraint_types(Upper(model)))
    @test !isempty(JuMP.list_of_constraint_types(Lower(model)))
    JuMP.delete(model, ctrl)
    @test isempty(JuMP.list_of_constraint_types(Lower(model)))
end

function constraint_dualof()
    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @constraint(Lower(model), ctrs[i in 1:2], y == 0)

    @test_throws ErrorException DualOf(ctrs)
end

function constraint_hints()
    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @constraint(Lower(model), lin, y == 0)
    @constraint(Lower(model), soc, [y, x] in SecondOrderCone())

    @test_throws ErrorException BilevelJuMP.set_dual_lower_bound_hint(lin, [1])
    @test_throws ErrorException BilevelJuMP.set_dual_upper_bound_hint(soc, 1)
    @test_throws ErrorException BilevelJuMP.set_dual_lower_bound_hint(soc, [1])
end
