
function moi_01(optimizer)

    # original problem
    # min -4x -3y
    # s.t.
    #      2x + y <= 4
    #       x +2y <= 4
    #       x     >= 0
    #           y >= 0
    #
    # sol x = y = 4/3
    # obj = 28/3

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    # add 10 variables - only diagonal is relevant
    X = MOI.add_variables(optimizer, 2)

    c1 = MOIU.normalize_and_add_constraint(
        optimizer,
        MOI.ScalarAffineFunction(
            [MOI.ScalarAffineTerm(2.0, X[1]), MOI.ScalarAffineTerm(1.0, X[2])],
            0.0,
        ),
        MOI.EqualTo(4.0),
    )

    c2 = MOIU.normalize_and_add_constraint(
        optimizer,
        MOI.ScalarAffineFunction(
            [MOI.ScalarAffineTerm(1.0, X[1]), MOI.ScalarAffineTerm(2.0, X[2])],
            0.0,
        ),
        MOI.EqualTo(4.0),
    )

    b1 = MOIU.normalize_and_add_constraint(
        optimizer,
        MOI.ScalarAffineFunction([MOI.ScalarAffineTerm(1.0, X[1])], 0.0),
        MOI.GreaterThan(0.0),
    )

    b2 = MOIU.normalize_and_add_constraint(
        optimizer,
        MOI.ScalarAffineFunction([MOI.ScalarAffineTerm(1.0, X[2])], 0.0),
        MOI.GreaterThan(0.0),
    )

    MOI.set(
        optimizer,
        MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(),
        MOI.ScalarAffineFunction(
            MOI.ScalarAffineTerm.([-4.0, -3.0], [X[1], X[2]]),
            0.0,
        ),
    )
    MOI.set(optimizer, MOI.ObjectiveSense(), MOI.MIN_SENSE)
    MOI.optimize!(optimizer)

    obj = MOI.get(optimizer, MOI.ObjectiveValue())

    @test obj ≈ -9.33333 atol = 1e-2

    Xr = MOI.get(optimizer, MOI.VariablePrimal(), X)

    @test Xr ≈ [1.3333, 1.3333] atol = 1e-2
end

function moi_02(optimizer, mode = BilevelJuMP.SOS1Mode())

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

    ### Build upper level model

    upper = MOIU.Model{Float64}()
    @test MOI.is_empty(upper)

    X = MOI.add_variables(upper, 2)

    MOI.set(upper, MOI.VariableName(), X[1], "x_u")
    MOI.set(upper, MOI.VariableName(), X[2], "y_u")

    MOI.set(
        upper,
        MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(),
        MOI.ScalarAffineFunction(
            MOI.ScalarAffineTerm.([-4.0, -3.0], [X[1], X[2]]),
            0.0,
        ),
    )
    MOI.set(upper, MOI.ObjectiveSense(), MOI.MIN_SENSE)

    ### Build lower level model

    lower = MOIU.Model{Float64}()
    @test MOI.is_empty(lower)

    X2 = MOI.add_variables(lower, 2)
    MOI.set(lower, MOI.VariableName(), X2[1], "x_l")
    MOI.set(lower, MOI.VariableName(), X2[2], "y_l")

    c1 = MOIU.normalize_and_add_constraint(
        lower,
        MOI.ScalarAffineFunction(
            [
                MOI.ScalarAffineTerm(2.0, X2[1]),
                MOI.ScalarAffineTerm(1.0, X2[2]),
            ],
            0.0,
        ),
        MOI.LessThan(4.0),
    )
    MOI.set(lower, MOI.ConstraintName(), c1, "l_c1")

    c2 = MOIU.normalize_and_add_constraint(
        lower,
        MOI.ScalarAffineFunction(
            [
                MOI.ScalarAffineTerm(1.0, X2[1]),
                MOI.ScalarAffineTerm(2.0, X2[2]),
            ],
            0.0,
        ),
        MOI.LessThan(4.0),
    )
    MOI.set(lower, MOI.ConstraintName(), c2, "l_c2")

    b1 = MOIU.normalize_and_add_constraint(
        lower,
        MOI.ScalarAffineFunction([MOI.ScalarAffineTerm(1.0, X2[1])], 0.0),
        MOI.GreaterThan(0.0),
    )
    MOI.set(lower, MOI.ConstraintName(), b1, "l_b1")

    b2 = MOIU.normalize_and_add_constraint(
        lower,
        MOI.ScalarAffineFunction([MOI.ScalarAffineTerm(1.0, X2[2])], 0.0),
        MOI.GreaterThan(0.0),
    )
    MOI.set(lower, MOI.ConstraintName(), b2, "l_b12")

    MOI.set(
        lower,
        MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(),
        MOI.ScalarAffineFunction(MOI.ScalarAffineTerm.([1.0], [X2[2]]), 0.0),
    )
    MOI.set(lower, MOI.ObjectiveSense(), MOI.MIN_SENSE)

    ### Build bilevel model combining both models

    links = Dict(X[1] => X2[1], X[2] => X2[2])

    parametric = [X2[1]]

    blp, _, _, _, _ =
        BilevelJuMP.build_bilevel(upper, lower, links, parametric, mode)

    # MOI.optimize!(bridged)

    # obj = MOI.get(bridged, MOI.ObjectiveValue())

    # @test obj ≈ -9.33333 atol = 1e-2

    # Xr = MOI.get(bridged, MOI.VariablePrimal(), X)

    # @test Xr ≈ [1.3333, 1.3333] atol = 1e-2

    # lp_model = MathOptFormat.MOF.Model()
    # MOI.copy_to(lp_model, blp)
    # MOI.write_to_file(lp_model, "my_model.LP")

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)
    MOI.copy_to(optimizer, blp)

    return MOI.optimize!(optimizer)
end

function moi_03(optimizer, mode = BilevelJuMP.SOS1Mode())
    upper = MOIU.Model{Float64}()
    @test MOI.is_empty(upper)

    x = MOI.add_variable(upper)
    y = MOI.add_variable(upper)

    MOI.set(upper, MOI.VariableName(), x, "x_u")
    MOI.set(upper, MOI.VariableName(), y, "y_u")

    MOI.set(
        upper,
        MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(),
        MOI.ScalarAffineFunction(
            MOI.ScalarAffineTerm.([1.0, 3.0], [x, y]),
            0.0,
        ),
    )
    MOI.set(upper, MOI.ObjectiveSense(), MOI.MIN_SENSE)

    lower = MOIU.Model{Float64}()
    @test MOI.is_empty(lower)

    x2 = MOI.add_variable(lower)
    y2 = MOI.add_variable(lower)

    MOI.set(lower, MOI.VariableName(), x2, "x_l")
    MOI.set(lower, MOI.VariableName(), y2, "y_l")

    c1 = MOIU.normalize_and_add_constraint(
        lower,
        MOI.ScalarAffineFunction(
            [MOI.ScalarAffineTerm(1.0, x2), MOI.ScalarAffineTerm(1.0, y2)],
            0.0,
        ),
        MOI.LessThan(8.0),
    )
    MOI.set(lower, MOI.ConstraintName(), c1, "l_c1")

    c2 = MOIU.normalize_and_add_constraint(
        lower,
        MOI.ScalarAffineFunction(
            [MOI.ScalarAffineTerm(1.0, x2), MOI.ScalarAffineTerm(4.0, y2)],
            0.0,
        ),
        MOI.GreaterThan(8.0),
    )
    MOI.set(lower, MOI.ConstraintName(), c2, "l_c2")

    c3 = MOIU.normalize_and_add_constraint(
        lower,
        MOI.ScalarAffineFunction(
            [MOI.ScalarAffineTerm(1.0, x2), MOI.ScalarAffineTerm(2.0, y2)],
            0.0,
        ),
        MOI.LessThan(13.0),
    )
    MOI.set(lower, MOI.ConstraintName(), c3, "l_c3")

    b1 = MOIU.normalize_and_add_constraint(
        lower,
        MOI.ScalarAffineFunction([MOI.ScalarAffineTerm(1.0, x2)], 0.0),
        MOI.GreaterThan(1.0),
    )
    MOI.set(lower, MOI.ConstraintName(), b1, "l_b1")

    b2 = MOIU.normalize_and_add_constraint(
        lower,
        MOI.ScalarAffineFunction([MOI.ScalarAffineTerm(1.0, x2)], 0.0),
        MOI.LessThan(6.0),
    )
    MOI.set(lower, MOI.ConstraintName(), b2, "l_b12")

    MOI.set(
        lower,
        MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(),
        MOI.ScalarAffineFunction(MOI.ScalarAffineTerm.([-1.0], [y2]), 0.0),
    )
    MOI.set(lower, MOI.ObjectiveSense(), MOI.MIN_SENSE)

    links = Dict(x => x2, y => y2)

    parametric = [x2]

    blp, _, _, _, _ =
        BilevelJuMP.build_bilevel(upper, lower, links, parametric, mode)

    # MOI.optimize!(bridged)

    # obj = MOI.get(bridged, MOI.ObjectiveValue())

    # @test obj ≈ -9.33333 atol = 1e-2

    # Xr = MOI.get(bridged, MOI.VariablePrimal(), X)

    # @test Xr ≈ [1.3333, 1.3333] atol = 1e-2

    # lp_model = MathOptFormat.MOF.Model()
    # MOI.copy_to(lp_model, blp)
    # MOI.write_to_file(lp_model, "my_model.LP")

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)
    MOI.copy_to(optimizer, blp)

    return MOI.optimize!(optimizer)
end
