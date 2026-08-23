# Copyright (c) 2019: Joaquim Dias Garcia, and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

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

    # `num_variables` is consistent with `all_variables` on each level
    @test JuMP.num_variables(model) == length(JuMP.all_variables(model))
    @test JuMP.num_variables(Upper(model)) ==
          length(JuMP.all_variables(Upper(model)))
    @test JuMP.num_variables(Lower(model)) ==
          length(JuMP.all_variables(Lower(model)))

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

function all_variables_levels()
    # an empty model has no variables in any level
    model = BilevelModel()
    @test isempty(JuMP.all_variables(model))
    @test isempty(JuMP.all_variables(Upper(model)))
    @test isempty(JuMP.all_variables(Lower(model)))

    # one variable of each possible level
    model = BilevelModel()
    @variable(Upper(model), x)      # UPPER_BOTH
    @variable(Lower(model), y)      # LOWER_BOTH
    @variable(UpperOnly(model), xo) # UPPER_ONLY
    @variable(LowerOnly(model), yo) # LOWER_ONLY
    @constraint(Lower(model), c, x + y <= 1)
    @variable(Upper(model), lam, DualOf(c)) # DUAL_OF_LOWER

    @test BilevelJuMP.mylevel(x) == BilevelJuMP.UPPER_BOTH
    @test BilevelJuMP.mylevel(y) == BilevelJuMP.LOWER_BOTH
    @test BilevelJuMP.mylevel(xo) == BilevelJuMP.UPPER_ONLY
    @test BilevelJuMP.mylevel(yo) == BilevelJuMP.LOWER_ONLY
    @test BilevelJuMP.mylevel(lam) == BilevelJuMP.DUAL_OF_LOWER

    # the bilevel model holds every variable, with no duplicates for the
    # linking variables that live in both levels
    vars = JuMP.all_variables(model)
    @test Set(vars) == Set([x, y, xo, yo, lam])
    @test length(vars) == 5
    @test allunique(vars)

    # each level only sees the variables that appear in it: the linking
    # variables `x` and `y`, plus the ones exclusive to that level
    @test Set(JuMP.all_variables(Upper(model))) == Set([x, y, xo, lam])
    @test Set(JuMP.all_variables(Lower(model))) == Set([x, y, yo])

    # `num_variables` agrees with `all_variables` on every level
    @test JuMP.num_variables(model) == length(JuMP.all_variables(model))
    @test JuMP.num_variables(Upper(model)) ==
          length(JuMP.all_variables(Upper(model)))
    @test JuMP.num_variables(Lower(model)) ==
          length(JuMP.all_variables(Lower(model)))

    # deleting a linking variable removes it from both levels
    JuMP.delete(model, x)
    @test Set(JuMP.all_variables(model)) == Set([y, xo, yo, lam])
    @test Set(JuMP.all_variables(Upper(model))) == Set([y, xo, lam])
    @test Set(JuMP.all_variables(Lower(model))) == Set([y, yo])

    # deleting a single level variable does not affect the other level
    JuMP.delete(model, yo)
    @test Set(JuMP.all_variables(model)) == Set([y, xo, lam])
    @test Set(JuMP.all_variables(Upper(model))) == Set([y, xo, lam])
    @test Set(JuMP.all_variables(Lower(model))) == Set([y])
    return nothing
end

struct _TestConstraintAttribute <: MOI.AbstractConstraintAttribute
    name::String
end

struct _TestVariableAttribute <: MOI.AbstractVariableAttribute
    name::String
end

# A minimal optimizer that supports arbitrary attributes, standing in for a
# solver (such as Gurobi) that supports them natively. It snapshots the
# attribute values visible at the start of the solve, which is what lets us
# assert that attributes set *before* `optimize!` reach the solver in time to
# affect it.
#
# This is a dedicated type rather than a method added to
# `MOI.Utilities.UniversalFallback`: BilevelJuMP itself builds a
# `CachingOptimizer` over a `UniversalFallback{Model{Float64}}`, so adding
# `MOI.optimize!` to that type would pirate it for every other test in the
# session.
mutable struct _TestSolver <: MOI.AbstractOptimizer
    inner::MOI.Utilities.UniversalFallback{MOI.Utilities.Model{Float64}}
    seen_ctr::Vector{Any}
    seen_var::Vector{Any}
    function _TestSolver()
        inner = MOI.Utilities.UniversalFallback(MOI.Utilities.Model{Float64}())
        return new(inner, Any[], Any[])
    end
end

# Forward the model API to the wrapped `UniversalFallback`.
MOI.is_empty(m::_TestSolver) = MOI.is_empty(m.inner)
MOI.empty!(m::_TestSolver) = MOI.empty!(m.inner)
function MOI.supports_incremental_interface(m::_TestSolver)
    return MOI.supports_incremental_interface(m.inner)
end
MOI.add_variable(m::_TestSolver) = MOI.add_variable(m.inner)
function MOI.add_constraint(
    m::_TestSolver,
    f::MOI.AbstractFunction,
    s::MOI.AbstractSet,
)
    return MOI.add_constraint(m.inner, f, s)
end
function MOI.supports_constraint(
    m::_TestSolver,
    ::Type{F},
    ::Type{S},
) where {F<:MOI.AbstractFunction,S<:MOI.AbstractSet}
    return MOI.supports_constraint(m.inner, F, S)
end
MOI.copy_to(m::_TestSolver, src::MOI.ModelLike) = MOI.copy_to(m.inner, src)

const _TestSolverAttribute = Union{
    MOI.AbstractConstraintAttribute,
    MOI.AbstractModelAttribute,
    MOI.AbstractOptimizerAttribute,
    MOI.AbstractVariableAttribute,
}

function MOI.get(m::_TestSolver, attr::_TestSolverAttribute, args...)
    return MOI.get(m.inner, attr, args...)
end
function MOI.set(m::_TestSolver, attr::_TestSolverAttribute, args...)
    return MOI.set(m.inner, attr, args...)
end
function MOI.supports(m::_TestSolver, attr::_TestSolverAttribute, args...)
    return MOI.supports(m.inner, attr, args...)
end

MOI.get(::_TestSolver, ::MOI.TerminationStatus) = MOI.OPTIMAL
MOI.get(::_TestSolver, ::MOI.ResultCount) = 0

# Record the attributes the solver can see as the solve begins.
function MOI.optimize!(m::_TestSolver)
    empty!(m.seen_ctr)
    empty!(m.seen_var)
    for (F, S) in MOI.get(m.inner, MOI.ListOfConstraintTypesPresent())
        for ci in MOI.get(m.inner, MOI.ListOfConstraintIndices{F,S}())
            v = MOI.get(m.inner, _TestConstraintAttribute("Lazy"), ci)
            v === nothing || push!(m.seen_ctr, v)
        end
    end
    for vi in MOI.get(m.inner, MOI.ListOfVariableIndices())
        v = MOI.get(m.inner, _TestVariableAttribute("Lazy"), vi)
        v === nothing || push!(m.seen_var, v)
    end
    return
end

function solver_attributes_unit()
    # Solver-specific variable/constraint attributes (such as Gurobi's
    # `ConstraintAttribute("Lazy")`) are cached and forwarded to the solver
    # right before the solve, but only for objects that have a direct
    # counterpart there, i.e. upper level ones.
    solver = _TestSolver()
    model = BilevelModel(() -> solver; mode = BilevelJuMP.ProductMode(1e-5))
    @variable(Upper(model), x)
    @variable(Lower(model), y)
    @variable(LowerOnly(model), z)
    @objective(Upper(model), Min, -x - 2y)
    @constraint(Upper(model), ux, x <= 10)
    @objective(Lower(model), Min, y + z)
    @constraint(Lower(model), ly, x + y + z <= 8)

    ctr_attr = _TestConstraintAttribute("Lazy")
    var_attr = _TestVariableAttribute("Lazy")

    # Lower level objects are reformulated away and must error, whether or not
    # the solver model has been built.
    @test_throws ErrorException MOI.set(ly, ctr_attr, 1)
    @test_throws ErrorException MOI.get(ly, ctr_attr)
    @test_throws ErrorException MOI.set(z, var_attr, 1)
    @test_throws ErrorException MOI.get(z, var_attr)

    # Querying an attribute that was never set cannot reach the solver yet.
    @test_throws ErrorException MOI.get(ux, ctr_attr)
    @test_throws ErrorException MOI.get(x, var_attr)

    # Set before the solve, which is the only useful moment for attributes
    # such as "Lazy". Reading back comes from the cache.
    MOI.set(ux, ctr_attr, 3)
    MOI.set(x, var_attr, 7)
    @test MOI.get(ux, ctr_attr) == 3
    @test MOI.get(x, var_attr) == 7

    optimize!(model)

    # The values were visible to the solver at the start of the solve.
    @test solver.seen_ctr == [3]
    @test solver.seen_var == [7]

    # After the build the values round-trip through the solver itself.
    @test MOI.get(ux, ctr_attr) == 3
    @test MOI.get(x, var_attr) == 7

    # A variable declared in the lower level but shared with the upper level
    # does exist in the solver, so it is forwarded too.
    MOI.set(y, var_attr, 5)
    @test MOI.get(y, var_attr) == 5

    # Overwriting keeps only the last value, and the cache is replayed on
    # every subsequent solve.
    MOI.set(ux, ctr_attr, 2)
    optimize!(model)
    @test solver.seen_ctr == [2]
    @test sort(solver.seen_var) == [5, 7]
    return
end
